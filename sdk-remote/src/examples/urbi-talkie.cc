/*
 * Copyright (C) 2005-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#include <libport/sys/time.h>
#include <libport/pthread.h>
#include <libport/windows.hh>
#include <libport/unistd.h>

#include <urbi/uclient.hh>
#include <urbi/uconversion.hh>

class SoundPipe
{
public:
  SoundPipe(urbi::UClient &r1, urbi::UClient &r2);
  urbi::UCallbackAction microNotify(int side, const urbi::UMessage &msg);
  urbi::UCallbackAction speakerNotify(int side, const urbi::UMessage &msg);

private:
  static const size_t stackSize = 4;
  static const size_t minSendSize = 2048;
  class SoundStack
  {
  public:
    std::list<urbi::USound> stack;
    int serverStackPos;
  };
  urbi::UClient *robot[2];
  SoundStack stack[2];
  pthread_mutex_t lock[2];;

  void trySend(int source);
};


SoundPipe::SoundPipe(urbi::UClient &r1, urbi::UClient &r2)
{
  pthread_mutex_init(&lock[0], 0);
  pthread_mutex_init(&lock[1], 0);
  robot[0]=&r1;
  robot[1]=&r2;

  stack[0].serverStackPos = 0;
  stack[1].serverStackPos = 0;
  r1.setCallback(*this, &SoundPipe::microNotify, 0, "mic");
  r2.setCallback(*this, &SoundPipe::microNotify, 1, "mic");
  r1.send("loop mic << micro.val,");
  r2.send("loop mic << micro.val,");
  r1.setCallback(*this, &SoundPipe::speakerNotify, 0, "speak");
  r2.setCallback(*this, &SoundPipe::speakerNotify, 1, "speak");
}


urbi::UCallbackAction
SoundPipe::microNotify(int source, const urbi::UMessage &msg)
{
  pthread_mutex_lock(&lock[source]);
  if (stack[source].stack.size() >= stackSize)
    {
      //drop it
      pthread_mutex_unlock(&lock[source]);
      return urbi::URBI_CONTINUE;
    }

  //convert it
  urbi::USound snd;
  snd.size = 0;
  snd.data = 0;
  snd.channels = 1;
  snd.rate = 16000;
  snd.sampleSize = 16;
  snd.sampleFormat = urbi::SAMPLE_SIGNED;


  urbi::USound &lastStacked = stack[source].stack.back();
  if (stack[source].stack.empty() || lastStacked.size > minSendSize)
    {
      snd.soundFormat = urbi::SOUND_RAW;
      convert(msg.value->binary->sound, snd);
      stack[source].stack.push_back(snd);
    }
  else
    {
      snd.soundFormat = urbi::SOUND_RAW;
      convert(msg.value->binary->sound, snd);
      lastStacked.data = (char *) realloc(lastStacked.data,
					  lastStacked.size + snd.size);
      memcpy(lastStacked.data + lastStacked.size, snd.data, snd.size);
      lastStacked.size += snd.size;
      printf("%d queed %zd\n", source, lastStacked.size);
      free(snd.data);
      snd.data=0;
    }

  pthread_mutex_unlock(&lock[source]);
  trySend(1-source);

  return urbi::URBI_CONTINUE;
}

urbi::UCallbackAction
SoundPipe::speakerNotify(int source, const urbi::UMessage &msg)
{
  if (msg.type != urbi::MESSAGE_SYSTEM || !strstr(msg.message.c_str(),"stop"))
    return urbi::URBI_CONTINUE;

  pthread_mutex_lock(&lock[source]);
  stack[source].serverStackPos--;
  pthread_mutex_unlock(&lock[source]);

  trySend(1-source);

  return urbi::URBI_CONTINUE;
}


void
SoundPipe::trySend(int source)
{
  pthread_mutex_lock(&lock[0]);
  pthread_mutex_lock(&lock[1]);

  if (stack[source].stack.empty()
      || stack[1-source].serverStackPos>=3
      || stack[source].stack.front().size<minSendSize)
    {
      pthread_mutex_unlock(&lock[1]);
      pthread_mutex_unlock(&lock[0]);
      return ;
    }

  urbi::USound snd = stack[source].stack.front();
  stack[source].stack.pop_front();
  stack[1-source].serverStackPos++;
  pthread_mutex_unlock(&lock[source]);
  pthread_mutex_unlock(&lock[1-source]);
  printf("%d sent %zd\n", source, snd.size);
  robot[1-source]->sendSound("speaker", snd,"speak");
  free(snd.data);
  snd.data = 0;
}


int main(int argc, char * argv[])
{
  if (argc<3)
  {
    fprintf(stderr,"usage: %s robot1 robot2\n\tplays what robot1 hears with robot2's speaker, and vice-versa\n",argv[0]);
    exit(1);
  }
  urbi::UClient * r[2];
  for (int i=0;i<2; ++i)
    {
      r[i]=new urbi::UClient(argv[i+1]);
      r[i]->start();
      if (r[i]->error())
	exit(1);
    }
  SoundPipe sp(*r[0], *r[1]);
  while (1)
    sleep(1);
}
