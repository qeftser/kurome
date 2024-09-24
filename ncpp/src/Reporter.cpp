
#include "Kurome.h"
#include <fcntl.h>

template class ll_queue<KB *>;

void reporter_client(Reporter * me) {
   
   gsock_fd broadcastfd;
   gsock_fd connfd = 0;

   gsockaddr_in broadcastaddr, connaddr, servaddr;
   broadcastaddr.sin_family = AF_INET;
   broadcastaddr.sin_addr.s_addr = htonl(INADDR_ANY);
   broadcastaddr.sin_port = htons(KUROME_BROADCAST_PORT);

   agent_discover discovery;
   struct timeval tv = { 0, 0 };

   broadcastfd = gsocket(AF_INET,SOCK_DGRAM,0);
   if (broadcastfd == -1)
      perror("gsocket");
   if (-1 == gbind(broadcastfd,(gsockaddr *)&broadcastaddr,sizeof(broadcastaddr)))
      perror("gbind");

   fd_set allrset, allwset;
   fd_set    rset,    wset;
   FD_ZERO(&allrset); FD_ZERO(&allwset);
   FD_SET(broadcastfd,&allrset);

   kurome_msg_transport rstate, wstate;

   int nready, nfds = broadcastfd, slen, n;

   for ( ; ; ) {
      //printf("client: looping\n");
      rset = allrset;
      if ((long)!me->reqs.empty()|rstate.known|wstate.known)
         wset = allwset;
      else
         FD_ZERO(&wset);
      bzero(&tv,sizeof(tv));
      tv.tv_sec = 5;
      nready = select(nfds+1,&rset,&wset,NULL,&tv);
      //printf("client: select triggered\n");

      if (FD_ISSET(broadcastfd,&rset)) {
         //printf("client: recieving broadcast\n");
         slen = sizeof(servaddr);
         recvfrom(broadcastfd,(void *)&discovery,sizeof(discovery),0,(gsockaddr *)&servaddr,(socklen_t *)&slen);
         int exists = 0;
         struct agent_values * curr = me->avaliable;
         while (curr) {
            if (curr->naddr == servaddr.sin_addr.s_addr)
               exists = 1;
            curr = curr->nextptr;
         }
         if (!exists) {
            //printf("client: new broadcast\n");
            struct agent_values * newv = (struct agent_values *)calloc(sizeof(struct agent_values),1);
            inet_ntop(AF_INET,&servaddr.sin_addr.s_addr,(char *)&newv->addr,(socklen_t)52);
            newv->naddr = servaddr.sin_addr.s_addr;
            newv->flags = discovery.flags;
            newv->port = discovery.start_port;
            newv->nextptr = me->avaliable;
            me->avaliable = newv;
         }
      }

      if (connfd) {
         if (FD_ISSET(connfd,&wset) && !me->reqs.empty()) {
            // handle writing data
            if (!wstate.known) {
               //printf("client: starting new write\n");
               KB * tosend;
               me->reqs.dequeue(&tosend);
               wstate.state.buf = tosend;
               wstate.state.nr = 0;
               wstate.state.len = tosend->size;
               wstate.known = 1;
            }

            //printf("client: writing\n");
            n = gnbwrite(connfd,&wstate.state);
            if (n == -1) {
               if (gerror() == GEWOULDBLOCK)
                  goto breakaway;
               wstate.known = 0;
               free(wstate.state.buf);
               perror("gnbwrite");
            }

            if (wstate.state.nr == wstate.state.len) {
               if (wstate.known == 1) {
                  //printf("client: fin writing\n");
                  free(wstate.state.buf);
                  bzero(&wstate,sizeof(wstate));
               }
            }
         }
breakaway:
         if (FD_ISSET(connfd,&rset)) {
            // handle reading data 
            if (!rstate.known) {
               //printf("client: starting new read\n");
               rstate.state.buf = malloc(sizeof(kurome_basemsg));
               rstate.state.nr = 0;
               rstate.state.len = sizeof(kurome_basemsg);
               rstate.known = 1;
            }

            //printf("client: reading\n");
            n = gnbread(connfd,&rstate.state);
            if (!n) {
               //printf("client: closing connection\n");
               gclose(connfd);
               return;
            }
            if (n == -1) {
               if (gerror() == GEWOULDBLOCK)
                  continue;
               rstate.known = 0;
               free(rstate.state.buf);
               perror("gnbread");
            }

            if (rstate.state.nr == rstate.state.len) {
               if (rstate.known == 1) {
                  //printf("client: base message read\n");
                  rstate.state.buf = realloc(rstate.state.buf,((KB *)rstate.state.buf)->size);
                  rstate.known = 2;
                  rstate.state.len = ((KB *)rstate.state.buf)->size;
               }
               else if (rstate.known == 2) {
                  //printf("client: full message read\n");
                  if (me->handlers.count(((KB *)rstate.state.buf)->type))
                     me->handlers.at(((KB *)rstate.state.buf)->type)((KB *)rstate.state.buf,me);
                  free(rstate.state.buf);
                  bzero(&rstate,sizeof(kurome_msg_transport));
                  //printf("client finished reading\n");
               }
            }
         }
      }
      else {
         if (me->conn) {
            //printf("client: opening connection\n");
            
            connfd = gsocket(AF_INET,SOCK_STREAM,0);
            if (-1 == connfd)
               perror("gsocket");

            connaddr.sin_family = AF_INET;
            connaddr.sin_port = me->conn->port;
            connaddr.sin_addr.s_addr = me->conn->naddr;

            if (-1 == gconnect(connfd,(gsockaddr *)&connaddr,sizeof(connaddr)))
               perror("gconnect");

            FD_SET(connfd,&allwset);
            FD_SET(connfd,&allrset);
            if (connfd > nfds)
               nfds = connfd;

            n = fcntl(connfd,F_GETFL,0);
            fcntl(connfd,F_SETFL,n|O_NONBLOCK);

            //printf("client: connected\n");
         }
      }
   }
}


void Reporter::launchClient() {
   cli = std::thread (reporter_client, this);
}

void Reporter::clientSend(KB * req) {
   reqs.enqueue(req);
}

void Reporter::kcmdConnect(long naddr) {
   struct agent_values * curr = avaliable;
   while (curr) {
      if (curr->naddr == naddr) {
         conn = curr;
         return;
      }
      curr = curr->nextptr;
   }
}

void Reporter::registerHandler(int mtype, void (* func)(struct kurome_basemsg *, Reporter *)) {
   if (handlers.count(mtype)) {
      handlers.erase(mtype);
   }
   handlers.emplace(mtype,func);
}

void Reporter::setDefaultHandlers(void) {
   registerHandler(KUROME_MSG_ENTITY,kurome_reporter_default_MSG_ADD_ENTITY_handler);
   registerHandler(KUROME_MSG_ADD_ENTITY,kurome_reporter_default_MSG_ADD_ENTITY_handler);
   registerHandler(KUROME_MSG_SET_IDX,kurome_reporter_default_MSG_SET_IDX_handler);
   registerHandler(KUROME_MSG_CLEAR,kurome_reporter_default_MSG_CLEAR_handler);
   registerHandler(KUROME_MSG_CHG_UNITSIZE,kurome_reporter_default_MSG_CHGUNITSIZE_handler);
   registerHandler(KUROME_MSG_CHG_XBLOCKS,kurome_reporter_default_MSG_CHG_XBLOCKS_handler);
   registerHandler(KUROME_MSG_CHG_YBLOCKS,kurome_reporter_default_MSG_CHG_YBLOCKS_handler);
   registerHandler(KUROME_MSG_SELF,kurome_reporter_default_MSG_CHGSELF_handler);
   registerHandler(KUROME_MSG_CHGSELF,kurome_reporter_default_MSG_CHGSELF_handler);
   registerHandler(KUROME_MSG_GOAL,kurome_reporter_default_MSG_CHGGOAL_handler);
   registerHandler(KUROME_MSG_CHGGOAL,kurome_reporter_default_MSG_CHGGOAL_handler);
   registerHandler(KUROME_MSG_SAMPLE,kurome_reporter_default_MSG_SAMPLE_handler);
   registerHandler(KUROME_MSG_WAITERINFO,kurome_reporter_default_MSG_WAITERINFO_handler);
   registerHandler(KUROME_MSG_MAPPERINFO,kurome_reporter_default_MSG_MAPPERINFO_handler);
   registerHandler(KUROME_MSG_GRID,kurome_reporter_default_MSG_GRID_handler);
   registerHandler(KUROME_MSG_FULLGRID,kurome_reporter_default_MSG_FULLGRID_handler);
}
