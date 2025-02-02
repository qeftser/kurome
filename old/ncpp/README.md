
# Kurome

Kurome as it is now is a set of classes and functions that provides a 
framework for building the mapping software for the robot. The design
allows the developer to focus on implimenting the mapping algorithms,
and not on worrying about environment representation or absorbing
sensor data. You still have to do some of that, but it is much less.

## Overview

The general structure of programs will look something like this:
```
int main(void) {
    
    /* define environment, hitbox, goal area */
    Grid env    = Grid( ... );
    Entity me   = Entity( ... );
    Entity goal = Entity( ... );

    /* setup waiters */
    Waiter * w1 = new <Waiter Class Child>( ... );
    Waiter * w1 = new <Waiter Class Child>( ... );
    ...
    Waiter * wN = new <Waiter Class Child>( ... );

    /* setup mapping algorithm */
    Mapper * m = new <Mapper Class Child>( ... );

    /* define agent and add elements */
    Agent agent = Agent(me,goal,*m,env);
    agent.waiters.push_back(w1);
    agent.waiters.push_back(w2);
    ...
    agent.waiters.push_back(wN);

    /* launch server */
    agent.launchServer( ... );

    /* commence mapping / update loop */
    while (1) {
        /* apply new sensor data */
        for (Waiter * w : agent.waiters)
            if (w->poll())
                agent.environment.apply(w->serve());

        ...
        /* update position based on movement */
        ...

        /* use mapping algorithm to get next movement */
        agent.mapper.callback( ... );
        Frame f = agent.mapper.nextPoint();

        ...
        /* communicate calculated movement to some other process */
        ...

        ...
        /* check goal condition / wait for input */
        ...

        /* process some requests from clients */
        agent.updateFromServer();

        /* repeat */
    }
}
```
    
There is a lot in there. It is all pretty straightforward though. First
we define the environment we are in. This is the line:
```
    Grid env = Grid( ... );
```
The full syntax for the grid definition is:
```
    Grid( < unit size >,
          < x axis size >,
          < y axis size > );
```
These are all floating point numbers. The unit size argument declares how 
wide and long each unit of the grid should be. The x axis and y axis size 
arguments determine the total width and length of the entire grid. The class
will take these arguments and create a matrix representation of the environment.    
    
Next is the definition of the Entities, goal and me:
```
    Entity me   = Entity( ... );
    Entity goal = Entity( ... );
```
The full syntax for the entity constructor is:
```
    Entity( < x position >,
            < y position >,
            < x width    >,
            < y width    >,
            < type       >,
            < weight/val > );
```
The arguments passed to the entity class are enough to represent an arbitrary
shape in realspace. The x position, y position, x width, and y width variables
are all floating point numbers. The type argument is basically an enum which
determines which shape to represent. Weight is used to define a weight for
collisions. The parameter names are pretty straightforward. Here are some
examples though:
```
    /* define a square at (1,1) that has a width of 2 */
    Entity square = Entity(1,1,2,2,ENTITY_TYPE_RECT,0);

    /* define a rectangle at (5,7) that has a 
     * width of 7 and a height of 5 */
    Entity rectangle = Entity(5,7,7,5,ENTITY_TYPE_RECT,0);

    /* define a circle at (20.243,3) with a radius of 2.7813 */
    Entity circle = Entity(20.243,3,2.7813,2.7813,ENTITY_TYPE_ELPS,0);

    /* define an ellipse at (3,9) with a width of 27 and a height of 81 */
    Entity ellipse = Entity(3,9,27,81,ENTITY_TYPE_ELPS,0);

```
    
The next lines of interest are the waiter definitions:
```
    Waiter * w1 = new <Waiter Class Child>( ... );
    Waiter * w1 = new <Waiter Class Child>( ... );
    ...
    Waiter * wN = new <Waiter Class Child>( ... );

```
There is no real full definition to share because the waiter is designed to
be extended to handle various sensors. This means that each waiter will likely
have different constructers. I will explain the methods that the waiter class
contains though:
```
    /* predefined */
    void     waiter.dish(Sample * sample);
    Sample * waiter.serve(void);
    int      poll(void);

    /* defined by child class */
    void     waiter.prepare(void);
```
The method dish takes in a sample from a sensor and stores it. The method
serve returns the latest dished sample. The dish method is designed to be
called inside the prepare method. The prepare method is a child-defined
method that produces the samples that will eventually be served. The user
must define this method. If the last sample has just been served, or if
there have been no samples dished, serve will return NULL. You can check for
and avoid this case by calling poll to see if there is a sample avaliable. This
behavior is shown in the start of the mapping loop:
```
    for (Waiter * w : agent.waiters)
        if (w->poll())
            agent.environment.apply(w->serve());
```
What this code is doing is going through all the waiters and serving the samples
that are avaliable to the Grid class that we instantiated earlier.    
    
I will digress and talk about samples now because I mentioned them a lot above.
If you are writing a Waiter child class, your main goal will be to coerce the 
sensor data into a Sample class. A sample consists of an entity that shows the
area the data was obtained from, a floating point number that represents the 
unit size, with the same meaning as unit size in Grid, and a matrix that holds the
actual data that the sensor found. Once the data is in this form, the Grid method
apply can be used as so:
```
    grid.apply(sample);
```
This uses the entity and unit size to map the matrix of sensor data onto the grid 
and update the mapping of the environment. This allows developers of the sensor
wrappers to focus solely on producing the most useful sample from the data, instead
of having to worry about how to represent the data in a map or with other sensors.
This system also ends up being pretty modular. The grid doesn't care what the sensor
is as long as it can produce a sample, and the waiter doesn't care what the grid does
or looks like, it just makes the sample.   
   
The next important code to cover is the mapper:
```
    Mapper * m = new <Mapper Class Child>( ... );

```
The mapper will also be a class that doesn't have a set constructor, as each mapper
will be created as a child of the main mapper class. This class impliments a few
critical functions:
```
    Frame nextPoint(void);
    void  callback(int flags);
```
The callback function provides info to the mapper about what has happed since it's 
last mapping (change in position, change of goal) via the flags argument. Callback
also allows the mapper to recompute or preform more computation to determine which movement 
will be the best in terms of reaching our goal. The nextPoint function returns a Frame
that describes what position the mapper thinks it would be best to move to next. It is expected
that this info can be passed to another process to facilitate the actual movement. I should
also mention that the mapper class works by mapping over the Grid that the waiters feed into.   
   
A brief explaination of Frames. They are like entities without the shapes, and just contain
a position and rotation. They also hold data like weight and num, and can be chained in linked
lists. These fields are useful for mapping algorithms. This class contains a bunch of useful
helper functions as well. If you do mapper implimentation you will use it a lot.

This code should make more sense now:
```
    /* use mapping algorithm to get next movement */
    agent.mapper.callback( ... );
    Frame f = agent.mapper.nextPoint();

    ...
    /* communicate calculated movement to some other process */
    ...

```
   
That is the basic overview of kurome. There are a lot of helpful methods and
such that I didn't cover. I should have an api and some examples done soon which will help
with that. Basically the developer will be responsible for writing the mapper and the waiters.
Waiters are just a wrapper/formatter over a sensor class, and they use the Sample class to
communicate with the grid. The mapper is the home of the actual mapping algorith, which 
should use the grid to map the robot. This is done a loop until the goal is reached or
some other situation occurs. It is all implimentation dependent. It is important to note that
Kurome doesn't define classes for actually interfacing with sensor or motors, and also doesn't
try to structure in what order or how often different things can be called. That is because those
features are expected to be handled by other parts of the program, not by kurome. By keeping things
modular, I expect that prototyping and simulations will be eaiser to write and perform.

## Client/Server interaction

### Startup

I didn't really cover it in the overview, but kurome also supports a simple model for client-server
interaction. This is handled throught the methods launchServer, updateFromServer, and sendAll in Agent,
and the class Reporter. Agent acts as the server and Reporter acts as the client. Once again, there
is no interface baked into this, all that is supplied is the low level connection setup and the 
hooks to be able to modify the actions taken when messages are sent and received. The basic behavior
is to call
```
    agent.launchServer( < port > , < flags > );
```
on startup to get the server up and running. Port defines the port that the agent is accepting
requests on, and flags advertizes the commands that the server is willing to accept. There is 
no need to save the port or flags, as the server will automatically broadcast it to all clients
on the subnet. From there the client program can define some way to look at the avaliable 
connections and connect the one it wants.   
   
On the client side we do:
``` 
    reporter.launchClient();

    ...
    /* somehow select the agent we want to connect to */
    ...

    reporter.kcmdConnect( < numerical ip addr > );
```
This gives us a connection to the agent.    
    
I will also discuss the Reporter class. Other than providing the client program, it holds a sort of replica
of the data in an Agent class. The goal of the reporter is to use the messages that the agent sends it
to construct a representation of what it thinks is happening in the agent.    

### Message passing

There a number of default messages and operations defined for passing data. It is pretty straightforward
to impliment new ones though, all it takes is some digging though the code :). The way messages are expected
to be passed between the Agent and Reporter are through their respective message queues. The reporter class
has an object called reqs, and the Agent has a reqs object for each open connection in a set called conns.
Any message enqueued in these queues will can be immediately considered to have been sent to its counterpart.
The developer doesn't have to care at all about how the client and server are going to be communicating, it
just needs to pass its messages to the message queue. Agent is similar but has things a little harder. 
Each connection has its own queue, so we need to be more careful how we send messages out.   
   
It is hard to explain this without examples. Basically the lower level communication operates on a struct
called kurome_basemsg to send and receive data. This struct contains the size of the total message and the
type of message. This isn't very useful on its own, so messages that need to send more than just a type
will have to define a struct that extends the basemsg struct. This is kind of mind-bending if you haven't
done it before, so there are already a good number of message types defined. Adding more is still pretty
simple though. Even with the structs there is still the issue of getting the classes to fit into the
message structs so they can be passed over a network. It is kind of a pain to do this, especially with
structs that have to be dynamically resized. Do deal with this there is a nice class called kcmd defined
that has a static method for all the message types. These calls will take the data in its normal class form, 
as well as the queue to add to. It will then do all the conversion and add the resulting message to the 
queue. This is the preferred way to send data. As an example, to send a entity we want added to the
environment from the Reporter, we would call:
```
    kcmd::addEntity(< our entity >, &reporter.reqs);
```
This would unbox our entity, format it, and send it off without us having to touch any of it. Pretty nice.   
   
Also, addressing the issue with multiple connections in the server, I have defined the method sendAll, which
is another wrapper over some of the kcmd functions that will iterate through all the open connections, sending 
the same message. For example, if we want to send our current grid to all connected clients, we could
use the call:
```
    agent.sendAll(< our grid >);
```

### Message processing

The above section explains how we go about sending our messages, but it doesn't explain what we do with them 
as the server or client. This behavior is defined through a set of message-based callbacks. These callbacks
are applied to messages automatically based the message type, and are called implicitly when the Reporter
recives a message. To get any messages processed in the Agent, you would need to call updateFromServer as such:
```
    /* process up to (< updates > - 1) messages */
    agent.updateFromServer(< updates >);

    /* default is 13, so process 12 messages
     * there is a limit to prevent a busy agent from
     * never returning from this call                */
    agent.updateFrmServer();
```

There are a number of useful callbacks defined for the Agent and Reporter classes, but new callbacks can be added
and existing callbacks can be changed quite easily. Both the Agent and Reporter have the function registerHandler,
which has the definition:
```
    /* reporter */
    reporter.registerHandler(< message type > ,  < function >);

    /* agent */
    agent.registerHandler(< message type > , < function >);
```
The function used in the registration call will end up being the one used to process every message of the given
type that passes through. This also allows the system to easily extend to process new messages. There are some
requirments on what functions can be passed though. Reporter requires the functions to have the template:
```
    < callback name >(struct kurome_basemsg * msg, Reporter * me);
```
The msg variable will need to be casted to the appropriate struct and then used. We do get access to the
full reporter, so we can update our state and send messages back automatically if we wanted to.    
    
For Agent the function needs to be of the form:
```
    < callback name >(struct kurome_basemsg * msg, ll_queue<KB *> * from, Agent * me);
```
Because the server has multiple connections it is important that it knows where a connection came from
in case it needs to reply. That is what the from variable is for, but other than that, the drill is 
the same. The hope here is to allow a good amount of flexability if it is needed.

### Closing statements

There are a couple of things to note from this design. One is that the Agent and reporter can be started in
the same process, same thread, if they needed to be. They spawn their connections in seperate threads so there
is no blocking or stalling in the user processes. Also, it should be noted that there is no interface for all
this defined. Like the rest of kurome, I have also opted for a more framework like approach for the communication
model. You could easily wrap a cli or gui over this if need be.

## Closing statements

More documentation to come. Please read through some of the code and tell me how it is looking though :).

## Todo

 - unit tests
 - rotations
 - negative coordinates
 - examples
 - api docs

## Name?

https://myanimelist.net/character/65297/Kurome/
