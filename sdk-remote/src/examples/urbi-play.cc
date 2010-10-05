/*
 * Copyright (C) 2005-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

#include <libport/csignal>
#include <libport/windows.hh>
#include <libport/unistd.h>
#include <libport/sysexits.hh>

#include <urbi/uclient.hh>

enum UType
{
  TYPE_BOOL,
  TYPE_ANGLE,
  TYPE_NORM
};
union UJointValue
{
  float angle;
  float normalized;
  bool boolean;
};
struct UCommand
{
  int timestamp;
  short id;
  UJointValue value;
};

struct UDev
{
  char * name;
  short id;
  UType type;
};
UDev* devices;
int devCount;
char dumpMode;

static bool
parseHeader(FILE *f)
{
  char buff[4];
  if (fread(buff,4,1,f)!=1)
    return false;
  if (strncmp(buff,"URBI",4))
    return false;
  if (fread(&devCount,4,1,f)!=1)
    return false;
  devices = new UDev[devCount];
  for (int i = 0; i < devCount; ++i)
  {
    char device[256];
    int pos=0;
    do {
      device[pos++] = fgetc(f);
      if (feof (f))
        return false;
    } while (device[pos-1]);
    devices[i].name = strdup(device);
    if (fread(&devices[i].id,2,1,f) != 1)
      return false;
    int type;
    if ((type=fgetc(f)) == EOF)
      return false;
    devices[i].type=(UType)type;
  }
  return true;
}

static void
play(urbi::UClient * robot, FILE *f)
{
  static int tick=0;
  static int ttime;

  UCommand uc;
  int starttime=0;
  bool commandPending = false;
  int lastCommandTime=0;
  while (fread(&uc,sizeof (uc),1,f)==1)
  {
    int sleeptime=uc.timestamp;
    if (!starttime)
    {
      starttime=robot->getCurrentTime()-sleeptime-1;
      lastCommandTime = 0;
    }
    int sleepstop=sleeptime+starttime; //when command should be executed in our timeframe
    //find the device
    UDev * dev=NULL;
    for (int i=0;i<devCount; ++i)
      if (devices[i].id==uc.id)
      {
        dev=&devices[i];
        break;
      }
    if (!dev)
    {
      fprintf(stderr,"device id %d not found\n",(int)uc.id);
      continue;
    }
    if (robot)
    {
      if (lastCommandTime != uc.timestamp)
      {
        if (commandPending)
          robot->send("noop;");
        if (sleepstop-robot->getCurrentTime() > 500)
          //queue no more than 500 ms in advance
          usleep((sleepstop-robot->getCurrentTime()-500)*1000);
      }
      commandPending = true;
      lastCommandTime = uc.timestamp;
      robot->send("%s.val = %f&",dev->name,uc.value.angle);
    }
    else
    {
      if (dumpMode=='-')
        printf("%d %s.val = %f\n",sleepstop, dev->name,uc.value.angle);
      else
      {
        if ( uc.timestamp!=lastCommandTime)
          printf("noop;\n");
        lastCommandTime = uc.timestamp;
        printf("%s.val = %f&\n", dev->name,uc.value.angle);
      }
    }
    if (!(tick%1000))
    {
      if (tick)
        fprintf( stderr,"%f cps\n",
                 1000000.0/(float)(robot->getCurrentTime()-ttime));
      ttime=robot->getCurrentTime();
    }
    ++tick;
  }

  if (robot && commandPending)
  {
    robot->send("noop;");
    commandPending = false;
  }
  if (!robot && dumpMode !='-')
    printf("noop;\n");
}


int main(int argc, char * argv[])
{
  if (argc<3)
  {
    printf(
      "usage: %s robot file [loop] \n"
      "\tPass '-' as 'robotname' to dump to stdout in human-readable format,\n"
      "\t or '+' to dump to stdout in urbi format.\n",
      argv[0]);
    exit(1);
  }

  int loop=0;
  if (argc>3)
  {
    loop=1;
    loop=strtol(argv[3],NULL,0);
  }

  urbi::UClient * robot;

  if ( (libport::streq(argv[1],"-")) || (libport::streq(argv[1],"+")))
  {
    robot=NULL;
    dumpMode=argv[1][0];
  }
  else
  {
    robot=new urbi::UClient(argv[1]);
    robot->start();
    if (robot->error())
      exit(4);
  }

  if (robot)
    robot->send("motoron;");

  do {
    FILE* f = libport::streq(argv[2],"-") ? stdin : fopen(argv[2],"r");
    if (!f)
      std::cerr << libport::format("error opening file `%s': %s\n",
                                   argv[2], strerror(errno))
                << libport::exit(EX_FAIL);
    if (!parseHeader(f))
      std::cerr << libport::format("error parsing header from `%s'\n",
                                   argv[2])
                << libport::exit(EX_FAIL);

    play(robot,f);
    fclose(f);
  } while (loop);
}
