
# vccp usage

Explanation of the various commands for the visual interface. This interface is still a work
in progress so it has a pretty minimal set of features currently. The GUI is built on the
sfml graphics library for c++. You will need a copy of the sfml library to compile this code.

sfml: https://www.sfml-dev.org/

## View

The grid is done in light grey. Squares with obstacles are shown in red, squares that are being mapped
are in yellow, and the chosen mapping is also in red. Entities are in green and are not mapped onto
the grid. 

## Movement

Movement is done through the arrow keys or h,j,k,l (vim). This will move the grid around on the screen.
The grid will automatically resize based on the window size.

## General

 * c - clear the entire grid
 * r - populate the grid with random shapes
 * m - map from the bottom left of the grid to the top right
 * x - cycle between generating a random set of shapes and slowly mapping through it

# Adding/WIP

You can think of the adding sequence in terms of sentences. For example the sentence "add
a rectangle that is 4 by 4 at (4,4)" is described by the command
```
ar4b4a4x4e
```
The syntax is in a tree:
```
  / c - - - - - - - \
a - r - <float> - b - <float> - a - <float> - x - <float> - e
   \ p - - - - - - - - - - - - - -/
```
Where 'a' stands for "add", 'r' is rectangle, 'c' is circle, and 'p' is point. 'b' is by, as 
in "3 by 5" when refering to size. 'x' and 'e' are just deliminators that work but don't make 
a ton of sense. :). Basically, plugging:
```
ac3a1.5x2.6e
```
in while in the program window will result in a circle of radius three being added at x position 1.5 and 
y position 2.6. Hopefully those examples explain the idea.   
   
     
Another feature that is partially working is the ability to click and drag entities around. Currently the
drag works but not the click. You can repeatedly click the spacebar as you slowly move your mouse as if you
were leading the shape, but that is janky. It is an issue with finding mouse actions on sfml, which I will
have to tackle at a later time.
