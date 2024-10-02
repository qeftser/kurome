
#include "Kurome.h"

#ifdef _WIN32

int ginet_pton(int protocol, char * src, char * dest) {
   return InetPtonW(protocol,src,dest);
}

int gaccept(gsock_fd s, gsockaddr * sa, int * addrlen) {
   return accept(s,sa,addrlen);
}

int gclose(gsock_fd s) {
   return closesocket(s);
}

int gerror(void) {
   return WSAGetLastError();
}

#else

int ginet_pton(int protocol, char * src, char * dest) {
   return inet_pton(protocol,src,dest);
}

int gaccept(gsock_fd s, gsockaddr * sa, int * addrlen) {
   return accept(s,sa,(socklen_t *)addrlen);
}

int gclose(gsock_fd s) {
   return close(s);
}

int gerror(void) {
   return errno;
}

#endif

gsock_fd gsocket(int af, int type, int protocol) {
   return socket(af,type,protocol);
}

int gconnect(gsock_fd s, const gsockaddr * sa, int len) {
   return connect(s,sa,len);
}

int gbind(gsock_fd s, const gsockaddr * sa, int len) {
   return bind(s,sa,len);
}

int glisten(gsock_fd s, int qsize) {
   return listen(s,qsize);
}

int gread(gsock_fd s, void * buf, int len) {
   return recv(s,buf,len,0);
}

int gwrite(gsock_fd s, void * buf, int len) {
   return send(s,buf,len,0);
}

int greadr(gsock_fd s, void * buf, size_t len) {
   int n;
   size_t nr = 0;
again:
   n = recv(s,(char *)buf+nr,len-nr,0);
   if (n == -1) {
      if (gerror() == GEINTR)
         goto again;
      return -1;
   }
   else if (!n)
      return 0;
   nr += n;
   if (nr != len)
      goto again;
   return nr;
}

int gwriter(gsock_fd s, void * buf, size_t len) {
   int n;
   size_t nr = 0;
again:
   n = send(s,(char *)buf+nr,len-nr,0);
   if (n == -1) {
      if (gerror() == GEINTR)
         goto again;
      return -1;
   }
   nr += n;
   if (nr != len)
      goto again;
   return nr;
}

int gnbread(gsock_fd s, gnbstate * nbs) {
   int n;
again:
   n = recv(s,(char *)nbs->buf+nbs->nr,nbs->len-nbs->nr,0);
   if (n == -1) {
      if (gerror() == GEINTR)
         goto again;
      return -1;
   }
   else if (!n)
      return 0;
   nbs->nr += n;
   if (nbs->nr != nbs->len)
      goto again;
   return nbs->nr;
}

int gnbwrite(gsock_fd s, gnbstate * nbs) {
   int n;
again:
   n = send(s,(char *)nbs->buf+nbs->nr,nbs->len-nbs->nr,0);
   if (n == -1) {
      if (gerror() == GEINTR)
         goto again;
      return -1;
   }
   nbs->nr += n;
   if (nbs->nr != nbs->len)
      goto again;
   return nbs->nr;
}
