
#ifndef GENERIC_SOCKET

#define GENERIC_SOCKET

#ifdef _WIN32

#include <Winsock2.h>
#include <ws2tcpip.h>

typedef SOCKET gsock_fd;
typedef sockaddr_in gsockaddr_in;
typedef sockaddr gsockaddr;

#define GEINTR       WSAEINTR
#define GEWOULDBLOCK WSAEWOULDBLOCK

#else

#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

typedef int gsock_fd;
typedef struct sockaddr_in gsockaddr_in;
typedef struct sockaddr gsockaddr;

#define GEINTR       EINTR
#define GEWOULDBLOCK EWOULDBLOCK

#endif

typedef struct GNBSTATE {
   void * buf;
   size_t nr;
   size_t len;
} gnbstate;

int gerror(void);

int ginet_pton(int protocol, void * src, void * dest);
gsock_fd gsocket(int af, int type, int protocol);
int gconnect(gsock_fd s, const gsockaddr * sa, int len);
int gbind(gsock_fd s, const gsockaddr * sa, int len);
int glisten(gsock_fd s, int qsize);
int gaccept(gsock_fd s, gsockaddr * sa, int * addrlen);
int gclose(gsock_fd s);
int gread(gsock_fd s, void * buf, int len);
int gwrite(gsock_fd s, void * buf, int len);
int gnbread(gsock_fd s, gnbstate * nbs);
int gnbwrite(gsock_fd s, gnbstate * nbs);



#endif
