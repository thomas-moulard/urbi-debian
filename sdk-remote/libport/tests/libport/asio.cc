/*
 * Copyright (C) 2008-2010, Gostai S.A.S.
 *
 * This software is provided "as is" without warranty of any kind,
 * either expressed or implied, including but not limited to the
 * implied warranties of fitness for a particular purpose.
 *
 * See the LICENSE file for more information.
 */

// If you experience failures in debug mode (singular iterator used),
// then you probably have a problem with Boost.  See
// http://groups.google.com/group/boost-list/browse_thread/thread/5eecec53397d0c52/05b272fb9a9000c5:
//
//   The code can be fixed (although there may be other occurrences of
//   similar problems) by changing around line 220 on asio/detail/
//   hash_map.hpp from:
//        // Update number of buckets and initialise all buckets to empty.
//        buckets_.resize(num_buckets);
//        for (std::size_t i = 0; i < buckets_.size(); ++i)
//          buckets_[i].first = buckets_[i].last = end;
//   to:
//        // Update number of buckets and initialise all buckets to empty.
//        bucket_type bucket;
//        bucket.first = bucket.last = end;
//        buckets_.resize(num_buckets, bucket);
//

#include <libport/test.hh>
#include <libport/sysexits.hh>

using libport::test_suite;

#include <libport/asio.hh>
#include <libport/lexical-cast.hh>
#include <libport/thread.hh>
#include <libport/utime.hh>
#include <libport/unistd.h>

bool abort_ctor = false;

const char* msg = "coincoin\n";

// On OSX:
//   listen            connect        status
// "127.0.0.1"       "127.0.0.1"       PASS  IPv4
// "localhost"       "localhost"       PASS  IPv6
// ""                "localhost"       PASS  IPv6
// ""                "127.0.0.1"       FAIL
// ""                ""                PASS  IPv6
static const char* listen_host = "127.0.0.1";
static const char* connect_host = "127.0.0.1";

template<class T>
void
hold_for(T, libport::utime_t duration)
{
  usleep(duration);
  BOOST_TEST_MESSAGE("Done holding your T");
}

void
delayed_response(boost::shared_ptr<libport::UDPLink> l,
                 libport::utime_t duration)
{
  usleep(duration);
  l->reply("hop hop\n");
}

// Hack until we can kill listenig sockets.
static bool enable_delay = false;
void reply_delay(const void*, int,
                 boost::shared_ptr<libport::UDPLink> l,
                 libport::utime_t duration)
{
  libport::startThread(boost::bind(&delayed_response, l, duration));
}

void echo(const void* d, int s, boost::shared_ptr<libport::UDPLink> l)
{
  if (enable_delay)
    reply_delay(d, s, l, 500000);
  else
    l->reply(d, s);
}

class TestSocket: public libport::Socket
{
  public:
  TestSocket()
    : nRead(0), echo(false), dump(false), destroyOnError(true)
    {
      BOOST_CHECK(!abort_ctor);
      nInstance++;
    }
  TestSocket(bool echo, bool dump)
    : nRead(0), echo(echo), dump(dump), destroyOnError(true)
    {
      BOOST_CHECK(!abort_ctor);
      nInstance++;
    }
  virtual ~TestSocket()
  {
    nInstance--;
    // BOOST_TEST_MESSAGE(this << " dying, in " << K_get()
    //                    << " lasterror=" << lastError.message());
    // aver(false);
  }

  size_t onRead(const void* data, size_t size)
  {
    nRead++;
    //BOOST_TEST_MESSAGE(this << " read " << size);
    if (echo)
      write(data, size);
    if (dump)
      received += std::string((const char*)data, size);
    return size;
  }

  void onError(boost::system::error_code erc)
  {
    lastError = erc;
    if (destroyOnError)
      destroy();
  }
  // Number of times read callback was called
  size_t nRead;
  // Echo back what is received.
  bool echo;
  // Store what is received in received.
  bool dump;
  // Destroy socket onError.
  bool destroyOnError;
  std::string received;
  boost::system::error_code lastError;
  static TestSocket* factory()
    {
      return lastInstance = new TestSocket();
    }
  static TestSocket* factoryEx(bool echo, bool dump)
    {
      return lastInstance = new TestSocket(echo, dump);
    }
  static size_t nInstance;
  // Last factory-created instance.
  static TestSocket* lastInstance;
};


size_t TestSocket::nInstance = 0u;
TestSocket* TestSocket::lastInstance = 0;

// Delay in microseconds used to sleep when something asynchronous
// is happening.
static const useconds_t delay = 200000;

static const int AVAIL_PORT = 7890;
static const std::string S_AVAIL_PORT = string_cast(AVAIL_PORT);

void test_one(bool proto)
{
  TestSocket* client = new TestSocket(false, true);
  boost::system::error_code err
    = client->connect(connect_host, S_AVAIL_PORT, proto);
  BOOST_REQUIRE_MESSAGE(!err, err.message());
  BOOST_CHECK_NO_THROW(client->send(msg));
  usleep(delay);
  BOOST_CHECK_EQUAL(TestSocket::nInstance, proto ? 1u : 2u);
  BOOST_CHECK_EQUAL(client->received, msg);
  if (!proto)
    BOOST_CHECK_EQUAL(TestSocket::lastInstance->received,
                      msg);
  BOOST_CHECK_EQUAL(client->getRemotePort(), AVAIL_PORT);
  if (!proto)
    BOOST_CHECK_EQUAL(TestSocket::lastInstance->getLocalPort(),
                      AVAIL_PORT);
  if (!proto)
    BOOST_CHECK_EQUAL(TestSocket::lastInstance->getRemotePort(),
                      client->getLocalPort());
  // Close client-end. Should send error both ways and destroy all sockets.
  client->close();
  usleep(delay);
  BOOST_CHECK_EQUAL(TestSocket::nInstance, 0u);
}


static
void
test()
{
  // Basic TCP
  BOOST_TEST_MESSAGE("##Safe destruction");
  boost::system::error_code err;
  TestSocket* s = new TestSocket(false, false);
  s->connect(connect_host, S_AVAIL_PORT, false);
  s->destroy();
  usleep(delay);
  libport::Socket* h = new libport::Socket();
  // Try listening on an IP that is not ours.
  BOOST_TEST_MESSAGE("Invalid IP in listen()");
  err = h->listen(boost::bind(&TestSocket::factoryEx, true, true),
	"1.2.3.4", "1212", false);
  BOOST_CHECK(err);
  err = h->listen(boost::bind(&TestSocket::factoryEx, true, true),
                  listen_host, S_AVAIL_PORT, false);
  BOOST_CHECK_MESSAGE(!err, err.message());
  BOOST_CHECK_EQUAL(h->getLocalPort(), AVAIL_PORT);

  BOOST_TEST_MESSAGE("##One client");
  test_one(false);
  BOOST_CHECK_EQUAL(h->getLocalPort(), AVAIL_PORT);
  test_one(false);
  BOOST_CHECK_EQUAL(h->getLocalPort(), AVAIL_PORT);
  test_one(false);
  BOOST_TEST_MESSAGE("Socket on stack");
  {
    TestSocket s(false, true);
    err = s.connect(connect_host, S_AVAIL_PORT, false);
    BOOST_CHECK_MESSAGE(!err, err.message());
    s.send(msg);
    usleep(delay);
  }
  usleep(delay*2);
  BOOST_CHECK_EQUAL(TestSocket::nInstance, 0u);

  test_one(false);
  BOOST_TEST_MESSAGE("##many clients");
  std::vector<TestSocket*> clients;
  for (int i=0; i<10; i++)
  {
    TestSocket* client = new TestSocket(false, true);
    err = client->connect(connect_host, S_AVAIL_PORT, false);
    BOOST_CHECK_MESSAGE(!err, err.message());
    client->send(msg);
    clients.push_back(client);
  }
  usleep(delay*3);
  BOOST_CHECK_EQUAL(TestSocket::nInstance, 20u);
  foreach(TestSocket* s, clients)
  {
    BOOST_CHECK_EQUAL(s->received, msg);
    s->close();
  }
  usleep(delay*3);
  BOOST_CHECK_EQUAL(TestSocket::nInstance, 0u);

  BOOST_TEST_MESSAGE("##Failing connections");
  {
    TestSocket* client = new TestSocket();
    err = client->connect("auunsinsr.nosuch.hostaufisgiu.com.", "20000", false);
    BOOST_CHECK_MESSAGE(err, err.message());

    err = client->connect(connect_host, "nosuchport", false);
    BOOST_CHECK_MESSAGE(err, err.message());

    // Try to reuse that wasted socket.
    err = client->connect(connect_host, S_AVAIL_PORT, false);
    BOOST_CHECK_MESSAGE(!err, err.message());
    // Destroy without closing.
    client->destroy();
    usleep(delay);
    BOOST_CHECK_EQUAL(TestSocket::nInstance, 0u);
  }

  // Timeout
  BOOST_TEST_MESSAGE("##Timeout connect");
  {
    TestSocket* client = new TestSocket();
    libport::utime_t start = libport::utime();
    err = client->connect("1.1.1.1", "10000", false, 1000000);
    BOOST_CHECK_MESSAGE(err, err.message());
    libport::utime_t timeout = libport::utime() - start;
    // Give it a good margin.
    BOOST_CHECK_LT(timeout, 1400000);
    client->destroy();
  }

  BOOST_TEST_MESSAGE("##Destruction locking");
  {
    TestSocket* client = new TestSocket();
    err = client->connect(connect_host, S_AVAIL_PORT, false);
    BOOST_CHECK_MESSAGE(!err, err.message());
    usleep(delay);
    libport::startThread(
      boost::bind(&hold_for<libport::Destructible::DestructionLock>,
                  client->getDestructionLock(),
                  1000000));
    client->destroy();
    usleep(500000);
    // There can be 1 or 2 sockets at this point. Client must be still
    // alive because of the lock, but server might have died.
    BOOST_CHECK_MESSAGE(TestSocket::nInstance, TestSocket::nInstance);
    usleep(500000+delay);
    BOOST_CHECK_EQUAL(TestSocket::nInstance, 0u);
  }

  BOOST_TEST_MESSAGE("Destroy listener");
  {
    h->close();
    usleep(delay);
    TestSocket* client = new TestSocket();
    abort_ctor = true;
    err = client->connect(connect_host, S_AVAIL_PORT, false);
    BOOST_CHECK_MESSAGE(err, err.message());
    client->destroy();
    h->destroy();
    usleep(delay);
    abort_ctor = false;
  }


  BOOST_TEST_MESSAGE("##UDP");
  {
    libport::Socket::listenUDP(listen_host, S_AVAIL_PORT, &echo, err);
    BOOST_CHECK_MESSAGE(!err, err.message());
    test_one(true);
    usleep(delay);
    test_one(true);

    TestSocket* client = new TestSocket();
    err = client->connect("auunsinsr.nosuch.hostaufisgiu.com.", "20000", true);
    BOOST_CHECK_MESSAGE(err, err.message());

    err = client->connect(connect_host, "nosuchport", true);
    BOOST_CHECK_MESSAGE(err, err.message());

    // Try to reuse that wasted socket.
    err = client->connect(connect_host, S_AVAIL_PORT, true);
    BOOST_CHECK_MESSAGE(!err, err.message());
    // Destroy without closing.
    client->destroy();
    usleep(delay);
    BOOST_CHECK_EQUAL(TestSocket::nInstance, 0u);

    // Check one-write-one-packet semantic
    client = new TestSocket(false, true);
    err = client->connect(connect_host, S_AVAIL_PORT, true);
    BOOST_CHECK_MESSAGE(!err, err.message());
    client->send("coin");
    client->send("pan");
    usleep(delay*2);
    BOOST_CHECK_EQUAL(client->received, "coinpan");
    BOOST_CHECK_EQUAL(client->nRead, 2u);

    enable_delay = true;
    client = new TestSocket(false, true);
    err = client->connect(connect_host, S_AVAIL_PORT, true);
    BOOST_CHECK_MESSAGE(!err, err.message());
    client->send("coin");
    usleep(500000+delay);
    BOOST_CHECK_EQUAL(client->received, "hop hop\n");
  }
}

static void test_pipe()
{
  TestSocket* s1 = new TestSocket(false, true);
  TestSocket* s2 = new TestSocket(false, true);
  s1->destroyOnError = false;
  s2->destroyOnError = false;
  libport::makePipe(std::make_pair(s2, s1));

  // First element is reader, second is writer.
  s1->send("canard");
  usleep(delay*2);
  BOOST_CHECK_EQUAL("canard", s2->received);
  s1->send("coin");
  usleep(delay*2);
  BOOST_CHECK_EQUAL(s2->received, "canardcoin");
  usleep(delay);
  // FIXME: fix the bug that SEGV when a pipe socket is destroyed.
  s1->destroy();
  s2->destroy();
  usleep(delay*2);
}

test_suite*
init_test_suite()
{
  skip_if("Qemu");
  skip_if("Wine");
  test_suite* suite = BOOST_TEST_SUITE("libport::asio test suite");
  suite->add(BOOST_TEST_CASE(test));
  suite->add(BOOST_TEST_CASE(test_pipe));
  return suite;
}
