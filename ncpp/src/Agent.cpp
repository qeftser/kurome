
#include "Kurome.h"
#include <fcntl.h>

/*
 * Connection server for kurome Agent class. Each
 * open client is handled by an instance of this
 * function. Mainly read messages coming in and
 * put them in the server queue, and write messages
 * in the client queue out.
 */
void kurome_agent_connection(gsock_fd fd, Agent * me) {
   printf("conn: launch\n");
   ll_queue<std::tuple<KB *,ll_queue<KB *> *>> * agent = &me->reqs;
   ll_queue<KB *> mydata;
   me->conns.insert(&mydata);

   fd_set lset, uset, wset, vset;
   FD_ZERO(&lset);
   FD_ZERO(&wset);
   FD_SET(fd,&lset);
   FD_SET(fd,&wset);
   int nready, n;
   kurome_msg_transport rstate, wstate;
   bzero(&rstate,sizeof(rstate));
   bzero(&wstate,sizeof(wstate));
   n = fcntl(fd,F_GETFL,0);
   fcntl(fd,F_SETFL,n|O_NONBLOCK);
   struct timeval tv = {0,0};

   for ( ; ; ) {
      printf("conn: loop start\n");
      uset = lset;
      if ((long)!mydata.empty()|rstate.known|wstate.known)
         vset = wset;
      else
         FD_ZERO(&vset);
      bzero(&tv,sizeof(tv));
      tv.tv_sec = 5;
      nready = select(fd+1,&uset,&vset,NULL,&tv);
      if (nready == -1)
         perror("select");

      printf("conn: select triggered\n");
      if (FD_ISSET(fd,&vset) && (!mydata.empty() || wstate.known)) {
         // handle writing data
         if (!wstate.known) {
            printf("conn: starting new write\n");
            KB * tosend;
            mydata.dequeue(&tosend);
            wstate.state.buf = tosend;
            wstate.state.nr = 0;
            wstate.state.len = tosend->size;
            wstate.known = 1;
         }

         printf("conn: writing\n");
         n = gnbwrite(fd,&wstate.state);
         if (n == -1) {
            if (gerror() == GEWOULDBLOCK)
               goto breakaway;
            wstate.known = 0;
            free(wstate.state.buf);
            perror("gnbwrite");
         }

         if (wstate.state.nr == wstate.state.len) {
            if (wstate.known == 1) {
               printf("conn: fin writing\n");
               free(wstate.state.buf);
               bzero(&wstate,sizeof(wstate));
            }
         }
      }
breakaway:
      if (FD_ISSET(fd,&uset)) {
         // handle reading data
         if (!rstate.known) {
            printf("conn: starting new read\n");
            rstate.state.buf = malloc(sizeof(kurome_basemsg));
            rstate.state.nr = 0;
            rstate.state.len = sizeof(kurome_basemsg);
            rstate.known = 1;
         }

         printf("conn: reading\n");
         n = gnbread(fd,&rstate.state);
         if (!n) {
            printf("conn: closing connection\n");
            gclose(fd);
            me->conns.erase(&mydata);
            KB * curr;
            while (mydata.dequeue(&curr)) {
               free(curr);
            }
            return;
         }
         if (n == -1) {
            if (gerror() == GEWOULDBLOCK)
               continue;
            rstate.known = 0;
            free(rstate.state.buf);
            perror("gnbread");
         }

check_readdone:
         if (rstate.state.nr == rstate.state.len) {
            if (rstate.known == 1) {
               printf("conn: base message read\n");
               rstate.state.buf = realloc(rstate.state.buf,((KB *)rstate.state.buf)->size);
               rstate.known = 2;
               rstate.state.len = ((KB *)rstate.state.buf)->size;
               goto check_readdone;
            }
            else if (rstate.known == 2) {
               printf("conn: full message read\n");
               KB * new_req = (KB *)rstate.state.buf;
               std::tuple res = std::make_tuple(new_req,&mydata);
               agent->enqueue(res);
               bzero(&rstate,sizeof(kurome_msg_transport));
               printf("conn: finished reading\n");
            }
         }
      }
   }
}

void kurome_agent_server(int port, Agent * me) {

   gsock_fd broadcastfd;
   gsock_fd listenfd;
   gsock_fd connfd;

   gsockaddr_in broadcastaddr;
   bzero(&broadcastaddr,sizeof(gsockaddr_in));
   broadcastaddr.sin_family = AF_INET;
   broadcastaddr.sin_addr.s_addr = 0xffffffff;
   broadcastaddr.sin_port = htons(KUROME_BROADCAST_PORT);
   struct agent_discover broadcastinfo;
   bzero(&broadcastinfo,sizeof(struct agent_discover));
   broadcastinfo.start_port = port;

   gsockaddr_in listenaddr;
   listenaddr.sin_family = AF_INET;
   listenaddr.sin_addr.s_addr = htonl(INADDR_ANY);
   listenaddr.sin_port = htons(port);

   gsockaddr_in connaddr;

   broadcastfd = gsocket(AF_INET,SOCK_DGRAM,0);
   if (-1 == broadcastfd)
      perror("socket");
   listenfd = gsocket(AF_INET,SOCK_STREAM,0);
   if (-1 == listenfd)
      perror("socket");

   if (-1 == gbind(listenfd,(gsockaddr *)&listenaddr,sizeof(gsockaddr_in)))
      perror("bind");
   if (-1 == glisten(listenfd,5))
      perror("listen");

   fd_set lset, uset;
   FD_ZERO(&lset);
   FD_SET(listenfd,&lset);

   int nready, connlen = 1;
   struct timeval tv = { 0, 0 };

   setsockopt(broadcastfd,SOL_SOCKET,SO_BROADCAST,&connlen,sizeof(connlen));

   for ( ; ; ) {
      printf("serv: loop start\n");
      uset = lset;
      bzero(&tv,sizeof(tv));
      tv.tv_sec = 5;
      nready = select(listenfd+1,&uset,NULL,NULL,&tv);
      if (nready && FD_ISSET(listenfd,&uset)) {
         printf("serv: accepting new connection\n");
         connlen = sizeof(connaddr);
         bzero(&connaddr,connlen);
         connfd = gaccept(listenfd,(gsockaddr *)&connaddr,&connlen);
         std::thread connection (kurome_agent_connection, connfd, me);
         connection.detach();
      }
      else {
         printf("serv: sending new broadcast\n");
         broadcastinfo.flags = me->flags;
         sendto(broadcastfd,&broadcastinfo,sizeof(struct agent_discover),0,(gsockaddr *)&broadcastaddr,sizeof(broadcastaddr));
      }
   }
}

void Agent::launchServer(int port, int flags) {
   flags = flags;
   serv = std::thread(kurome_agent_server, port, this);
   return;
}

void Agent::updateFromServer(int updates) {
   std::tuple<KB *,ll_queue<KB *> *> lreqs;
   while (reqs.dequeue(&lreqs) && --updates) {
      if (handlers.count(std::get<0>(lreqs)->type))
         handlers.at(std::get<0>(lreqs)->type)(std::get<0>(lreqs),std::get<1>(lreqs),this);
   }
}

double Agent::goalDist(void) {
   return std::sqrt(self.dist2(goal));
}

void Agent::registerHandler(int mtype, void (* func)(struct kurome_basemsg *, ll_queue<KB *> *, Agent *)) {
   if (handlers.count(mtype)) {
      handlers.erase(mtype);
   }
   handlers.emplace(mtype,func);
}

void Agent::setDefaultHandlers(void) {
   registerHandler(KUROME_MSG_ADD_ENTITY,kurome_agent_default_MSG_ADD_ENTITY_handler);
   registerHandler(KUROME_MSG_MAPCALLBACK,kurome_agent_default_MSG_MAPCALLBACK_handler);
   registerHandler(KUROME_MSG_SET_IDX,kurome_agent_default_MSG_SET_IDX_handler);
   registerHandler(KUROME_MSG_CLEAR,kurome_agent_default_MSG_CLEAR_handler);
   registerHandler(KUROME_MSG_CHG_UNITSIZE,kurome_agent_default_MSG_CHGUNITSIZE_handler);
   registerHandler(KUROME_MSG_CHG_XBLOCKS,kurome_agent_default_MSG_CHG_XBLOCKS_handler);
   registerHandler(KUROME_MSG_CHG_YBLOCKS,kurome_agent_default_MSG_CHG_YBLOCKS_handler);
   registerHandler(KUROME_MSG_GET_GRID,kurome_agent_default_MSG_GET_GRID_handler);
   registerHandler(KUROME_MSG_GET_FULLGRID,kurome_agent_default_MSG_GET_FULLGRID_handler);
   registerHandler(KUROME_MSG_CHGSELF,kurome_agent_default_MSG_CHGSELF_handler);
   registerHandler(KUROME_MSG_CHGGOAL,kurome_agent_default_MSG_CHGGOAL_handler);
   registerHandler(KUROME_MSG_CHGFLAGS,kurome_agent_default_MSG_CHGFLAGS_handler);
}

void Agent::sendAll(Entity & e) {
   for (ll_queue<KB *> * q : conns) {
      kcmd::entity(e,q);
   }
}

void Agent::sendAll(Entity & e, int mtype) {
   switch (mtype) {
      case KUROME_MSG_ADD_ENTITY:
         for (ll_queue<KB *> * q : conns) {
            kcmd::addEntity(e,q);
         }
         break;
      case KUROME_MSG_ENTITY:
         for (ll_queue<KB *> * q : conns) {
            kcmd::entity(e,q);
         }
         break;
      case KUROME_MSG_GOAL:
         for (ll_queue<KB *> * q : conns) {
            kcmd::goal(e,q);
         }
         break;
      case KUROME_MSG_CHGGOAL:
         for (ll_queue<KB *> * q : conns) {
            kcmd::chgGoal(e,q);
         }
         break;
      case KUROME_MSG_SELF:
         for (ll_queue<KB *> * q : conns) {
            kcmd::self(e,q);
         }
         break;
      case KUROME_MSG_CHGSELF:
         for (ll_queue<KB *> * q : conns) {
            kcmd::chgSelf(e,q);
         }
         break;
      default:
         for (ll_queue<KB *> * q : conns) {
            kcmd::entity(e,q);
         }
         break;
   }
}

void Agent::sendAll(Sample & s) {
   for (ll_queue<KB *> * q : conns) {
      kcmd::sample(s,q);
   }
}

void Agent::sendAll(Grid & g) {
   for (ll_queue<KB *> * q : conns) {
      kcmd::grid(g,q);
   }
}

void Agent::sendAll(Grid & g, int mtype) {
   switch (mtype) {
      case KUROME_MSG_GRID:
         for (ll_queue<KB *> * q : conns) {
            kcmd::grid(g,q);
         }
         break;
      case KUROME_MSG_FULLGRID:
         for (ll_queue<KB *> * q : conns) {
            kcmd::fullGrid(g,q);
         }
         break;
      default:
         for (ll_queue<KB *> * q : conns) {
            kcmd::grid(g,q);
         }
         break;
   }
}
