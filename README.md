# Kurome

dynamic map generation/update/navigation using arbitrary granularity

**Examples at the bottom!**

## Description

the last few meetings I have been a part of we discussed varying the 
amount of granularity we use in the mapping, as well as being able to
map the robot to back up if it doesn't have the turning radius to get
around an obstacle. This set of classes can preform both of those actions.

I haven't done it yet, but I will bind it up to python and ros2 if we think
it would be useful. 

## Usage

Create a Grid object
```
Grid * g = new Grid( < unit ( width/height of one square in the grid ) > ,
                     < x size (not in units) > ,
                     < y size (not in units) > );
```

So making a grid with
```
Grid * g = new Grid( 1, 25.27, 23.45);
```
or
```
Grid * g = new Grid( 0.1, 25.27, 23.45);
```
doesn't change the size of the grid, but it does increase the number of squares
or nodes (the granularity) of the grid.   
   
    
Create an entity
```
/*

Entity * e = new Entity( < danger > , < type > , < width/radius > ,
                         < height > , < x pos > , < y pos > );
*/

/* Rectangle centered at (3,5) with area of 12 */
Entity * rect = new Entity(10,ENTITY_TYPE_RECT,3,4,3,5);

/* Circle with radius of 5 at (8,23.923) */
Entity * circ = new Entity(24,ENTITY_TYPE_RECT,5,-1,8,23.923);

/* Point at (4.20,6.96969) */
Entity * point = new Entity(28,ENTITY_TYPE_POINT,-1,-1,4.20,6.96969);
```

These represent obstacles on the map. Note the positions and sizes can be at any accuracy,
they are not tied to the grid unit size at all. The only types are rect,circle,and point, defined
in the way you see above. The danger variable indicates the perceived danger of the location, how
much we want to avoid it.   
   
   
A subclass of the entity is WeightedEntity:
```
/*

WeightedEntity * e = new WeightedEntity(< danger > , < type > , < width/radius > ,
                                        < height > , < x pos >, < y pos > , < weight >);
*/

/* Same rectangle as above but weighted 0.9 */
Entity * w_rect = new WeightedEntity(10,ENTITY_TYPE_RECT,3,4,3,5,0.9); 

/* weight is between 0.0 and 1.0 */
```

The weight changes the preference given to the danger in the entity when it is added to the grid 
and the danger is combined with the danger value that the nodes previously had. In the normal entities
a default value of 0.6 is used and is held in the macro **GRID_NEW_VALUE_WEIGHT**.   
   

Add an entity to the grid
```
g->addEntity(rect);
g->addEntity(circ);
g->addEntity(point);
g->addEntity(w_rect);
```

This works for any entity type. It records the entity and then maps it to the grid based on the
unit size of the grid. It will round out for shapes, so the grid squares covered will always be
over a larger area than the entity's actual area. The smaller the unit size, the more accurate the
representation will be. One note here is that when filling the circles the macro **GRID_CIRCLE_GRANULARITY**
is used in the fill algorithm. This is set to 0.025 and should always be less than the granularity. Also
I will mention that the program detects and avoids mapping out of bounds of a grid, so you don't have to
worry about staying in bounds.   
   

Create an entity and map it:
```
/*

MovingEntity * me = new MovingEntity( /* ... same as normal entity ... */ , < turn radius > );

*/

/* a circle with a turning radius of 1 and a radius of 0.6. Sitting at (1.0,1.0) */
MovingEntity * my_ufo = new MovingEntity(-1,ENTITY_TYPE_CIRCLE,0.6,-1,1,1,1.0);

/* map and display the mapping to (5.0,5.0) */
g->mapEntity(my_ufo);
```

The entity size, shape, and turning radius are taken into account in the mapping. There are some parameters to note in the
computation. **GRID_MAP_FRAME_SLICES** determines the number of different angles that are attempted in the search algorithm, 
which is a modified A*. Roughly 4 \* **GRID_MAP_FRAME_SLICES** are created each step, increase this variable for higher angle
granularity. **GRID_MAP_FRAME_STEP** is used to determine the distance moved in each step. You can decrease it for finer grained 
movement, but if you do it too low the floating point calculations will go to zero and the mapping will fail. Finally, 
**GRID_MAP_ANGLE_SLICING** is used when pruning for already visited positions and angles. Unlike normal A*, we need to keep track
of the angle we are facing along with our position, and this is taken into account when checking for visited nodes. The higher
this value is the less variation will be accepted in angle differences. The calculation is done (currAngle/GRID_MAP_ANGLE_SLICING)
in degrees, so values 35 and 43 would be considered different when the parameter is 5 or 10 but the same when the parameter is 15.

## Examples
A square of width 2 centered in a 10x10 grid of granularity 2:
```
/* ex1.cpp */

int main(void) {


   Grid * g = new Grid(2,10,10);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,2,2,5,5);

   g->addEntity(e);
   g->print();

   return 0;
}
```
![ex1](https://github.com/user-attachments/assets/68575122-e1fa-40e9-aced-72784beefbc0)
Apologies, I was too lazy to use an actual graphics library. Same square and grid size, but a granularity of 1:
```
/* ex2.cpp */

int main(void) {


   Grid * g = new Grid(1,10,10);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,2,2,5,5);

   g->addEntity(e);
   g->print();

   return 0;
}
```
![ex2](https://github.com/user-attachments/assets/8f535b56-4632-4125-9cad-2ebfa93e85d1)
Now the granularity at 0.1:
```
/* ex3.cpp */

int main(void) {


   Grid * g = new Grid(0.1,10,10);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,2,2,5,5);

   g->addEntity(e);
   g->print();

   return 0;
}
```
![ex3](https://github.com/user-attachments/assets/024f018a-e4c5-4a21-a2b2-aab86a08ce36)
Hopefully that does a good job of showing the changes. The numbers are the danger/weight of each
grid section in hex. Here is the map I will use to show the mapping. The origin and size of our entity
is in magenta and the destination is green:
```
/* ex4.cpp */

int main(void) {

   Grid * g = new Grid(0.10,30.0,20.0);

   MovingEntity * e3 = new MovingEntity(0xa3,ENTITY_TYPE_RECT,1.0,1.0,2.0,2.0,0.5);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,1.5,1.5,6.0,6.0);
   Entity * e2 = new Entity(0x2f,ENTITY_TYPE_RECT,4.0,4.0,6.0,6.0);
   Entity * e4 = new Entity(0x2f,ENTITY_TYPE_CIRCLE,4.0,1.0,10.0,10.0);
   Entity * e5 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,8.0,10.0,1.0);
   Entity * e6 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,4.5,13.0,18.0);
   Entity * e7 = new Entity(0x08,ENTITY_TYPE_RECT,2.0,2.0,18.0,4.0);
   g->addEntity(e);
   g->addEntity(e2);
   g->addEntity(e4);
   g->addEntity(e5);
   g->addEntity(e6);
   g->addEntity(e7);
   g->addEntity(e3);

   g->print();

   return 0;
}
```
![ex4](https://github.com/user-attachments/assets/e19218e0-246c-4c0c-aafb-884b50d08359)
Now we will attempt to map to the destination. Values shaded yellow/orange are explored positions and the magenta
is the final chosen route:
```
/* ex5.cpp */

int main(void) {

   Grid * g = new Grid(0.10,30.0,20.0);

   MovingEntity * e3 = new MovingEntity(0xa3,ENTITY_TYPE_RECT,1.0,1.0,2.0,2.0,0.5);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,1.5,1.5,6.0,6.0);
   Entity * e2 = new Entity(0x2f,ENTITY_TYPE_RECT,4.0,4.0,6.0,6.0);
   Entity * e4 = new Entity(0x2f,ENTITY_TYPE_CIRCLE,4.0,1.0,10.0,10.0);
   Entity * e5 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,8.0,10.0,1.0);
   Entity * e6 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,4.5,13.0,18.0);
   g->addEntity(e);
   g->addEntity(e2);
   g->addEntity(e4);
   g->addEntity(e5);
   g->addEntity(e6);

   g->mapEntity(e3,18.0,4.0);

   return 0;
}
```
![ex5](https://github.com/user-attachments/assets/1416ca4a-edc7-49dc-a940-0df634afa63c)
Doubling the size of the entity gives:
```
/* ex6.cpp */

int main(void) {

   Grid * g = new Grid(0.10,30.0,20.0);

   MovingEntity * e3 = new MovingEntity(0xa3,ENTITY_TYPE_RECT,2.0,2.0,2.0,2.0,0.5);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,1.5,1.5,6.0,6.0);
   Entity * e2 = new Entity(0x2f,ENTITY_TYPE_RECT,4.0,4.0,6.0,6.0);
   Entity * e4 = new Entity(0x2f,ENTITY_TYPE_CIRCLE,4.0,1.0,10.0,10.0);
   Entity * e5 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,8.0,10.0,1.0);
   Entity * e6 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,4.5,13.0,18.0);
   g->addEntity(e);
   g->addEntity(e2);
   g->addEntity(e4);
   g->addEntity(e5);
   g->addEntity(e6);

   g->mapEntity(e3,18.0,4.0);

   return 0;
}
```
![ex6](https://github.com/user-attachments/assets/6a1d609a-6943-4cac-9d97-faac2deb23e4)
Now back to the old size, we will increase the turning radius from 0.5 to 2.00:
```
/* ex7.cpp */

int main(void) {

   Grid * g = new Grid(0.10,30.0,20.0);

   MovingEntity * e3 = new MovingEntity(0xa3,ENTITY_TYPE_RECT,1.0,1.0,2.0,2.0,1.00);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,1.5,1.5,6.0,6.0);
   Entity * e2 = new Entity(0x2f,ENTITY_TYPE_RECT,4.0,4.0,6.0,6.0);
   Entity * e4 = new Entity(0x2f,ENTITY_TYPE_CIRCLE,4.0,1.0,10.0,10.0);
   Entity * e5 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,8.0,10.0,1.0);
   Entity * e6 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,4.5,13.0,18.0);
   g->addEntity(e);
   g->addEntity(e2);
   g->addEntity(e4);
   g->addEntity(e5);
   g->addEntity(e6);

   g->mapEntity(e3,18.0,4.0);

   return 0;
}
```
![ex7_001](https://github.com/user-attachments/assets/9d52b62f-047c-4ca4-b312-db0d185d6964)
Here you can see that the entity was unable to make it around the corner and through that gap. When it
reaches that situation, it drives straight back until it has the proper angle and then proceeds through the gap
to the destination. You can see that the pathfinding is not perfect as well. It overshoots the destination
and attempts to go backwards there. You can see some of the pathfinding issues if we add another obstacle
in the path of our reverse:
```
/* ex8.cpp */

int main(void) {

   Grid * g = new Grid(0.10,30.0,20.0);

   MovingEntity * e3 = new MovingEntity(0xa3,ENTITY_TYPE_RECT,1.0,1.0,2.0,2.0,2.00);

   Entity * e = new Entity(0x2f,ENTITY_TYPE_RECT,1.5,1.5,6.0,6.0);
   Entity * e2 = new Entity(0x2f,ENTITY_TYPE_RECT,4.0,4.0,6.0,6.0);
   Entity * e4 = new Entity(0x2f,ENTITY_TYPE_CIRCLE,4.0,1.0,10.0,10.0);
   Entity * e5 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,8.0,10.0,1.0);
   Entity * e6 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,4.5,13.0,18.0);
   Entity * e7 = new Entity(0x2f,ENTITY_TYPE_RECT,2.0,2.5,5.0,18.0);
   g->addEntity(e);
   g->addEntity(e2);
   g->addEntity(e4);
   g->addEntity(e5);
   g->addEntity(e6);
   g->addEntity(e7);

   g->mapEntity(e3,18.0,4.0);

   return 0;
}
```
![ex8](https://github.com/user-attachments/assets/fa6c6087-2a04-4dfe-b8aa-71c7b145c500)
That is all for now. You can compile the examples using make ex\[n\] (make ex1) and run the executables
placed in ./bin. Also you can do make all (to make them all :))

## name?

https://myanimelist.net/character/65297/kurome





