# The Reddit Gold Universe - Hanyu Liu (liuhanyu)

## Inspiration
![](media.png)

## Live Demo Link 

https://helenl9098.github.io/hw02-raymarching-sdfs/

## Implementation Details

- __Geometry__: The Reddit robot and its surrounding orbits are made up of SDFs, combined together using Intersection, Union, Smooth Blend, and Difference operations.


- __Animation__: The robot is animated to bob up and down using a modified sin function. The scaling of the rings is animated using a modified sin function. The positional changes of the orbiting sphere is animated using a mod function on polar coordinates.


- __Texture / Shading__: The gold texture / shading of the robot is made from cosine color curves determined from the normals of the SDF


- __Optimization__: The scene is optimized using bounding volumes: the robot and the rings are each bound by their own bounding sphere. If the ray doesn't intersect with the bounding sphere, then it won't compute the SDF. If it intersects the bounding sphere, then it looks at the SDF of that sphere. 


## Modifiable Features

- __Sphere Stickiness__: The blend factor of the spheres can be adjusted, making them look either softer or more solid

- __Bounce Speed__: The bobbing speed of the Reddit robot can be adjusted, making him bounce faster or slower


## External Resources

- https://www.iquilezles.org/www/articles/distfunctions/distfunctions.htm
- http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
- https://iquilezles.org/www/articles/palettes/palettes.htm
