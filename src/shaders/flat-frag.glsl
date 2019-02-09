#version 300 es
precision highp float;

uniform vec3 u_Eye, u_Ref, u_Up;
uniform vec2 u_Dimensions;
uniform float u_Time;
uniform float u_Stickiness;
uniform float u_Bounce;

in vec2 fs_Pos;
out vec4 out_Col;

// based off of http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
float intersectSDF(float distA, float distB) {
    return max(distA, distB);
}

float unionSDF(float distA, float distB) {
    return min(distA, distB);
}

float differenceSDF(float distA, float distB) {
    return max(distA, -distB);
}

// based off of http://iquilezles.org/www/articles/distfunctions/distfunctions.htm
float opSmoothUnion( float d1, float d2, float k ) {
    float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) - k*h*(1.0-h); 
}

float opSmoothIntersection( float d1, float d2, float k ) {
    float h = clamp( 0.5 - 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) + k*h*(1.0-h); 
}


mat4 rotateX(float theta) {
    float c = cos(radians(theta));
    float s = sin(radians(theta));

    return mat4(
        vec4(1, 0, 0, 0),
        vec4(0, c, s, 0),
        vec4(0, -s, c, 0),
        vec4(0, 0, 0, 1)
    );
}

mat4 rotateZ(float theta) {
    float c = cos(radians(theta));
    float s = sin(radians(theta));

    return mat4(
        vec4(c, s, 0, 0),
        vec4(-s, c, 0, 0),
        vec4(0, 0, 1, 0),
        vec4(0, 0, 0, 1)
    );
}

float parabola (float x, float k) {
  return pow(4.0 * x * (1.0 - x), k);
}

vec3 getRayDirection() {

  float fovy = 30.0;
  vec3 look = normalize(u_Ref - u_Eye);
  vec3 right = normalize(cross(look, u_Up));
  vec3 up = cross(right, look);

  float tan_fovy = tan(radians(fovy / 2.0));
  float len = length(u_Ref - u_Eye);
  float aspect = u_Dimensions.x / float(u_Dimensions.y);

  vec3 v = up * len * tan_fovy;
  vec3 h = right * len * aspect * tan_fovy;

  vec3 p = u_Ref + fs_Pos.x * h + fs_Pos.y * v;
  vec3 dir = normalize(p - u_Eye);

  return dir;

}

float sdBox( vec3 p, vec3 b )
{
  vec3 d = abs(p) - b;
  return length(max(d,0.0))
         + min(max(d.x,max(d.y,d.z)),0.0); // remove this line for an only partially signed sdf 
}

float sdEllipsoid( in vec3 p, in vec3 r )
{
    float k0 = length(p/r);
    float k1 = length(p/(r*r));
    return k0*(k0-1.0)/k1;
}

float sdTorus( vec3 p, vec2 t )
{
  vec2 q = vec2(length(p.xz)-t.x,p.y);
  return length(q)-t.y;
}

float sdSphere( vec3 p, float s )
{
  return length(p)-s;
}

float sdCappedCylinder( vec3 p, vec2 h )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - h;
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

vec2 calcCircle( float theta, float radius) {
  float y = radius * sin(radians(theta)); 
  float x = radius * cos(radians(theta));
  return vec2(-x, -y - 0.7);
}


struct BoundingSphere {
  float radius;
  vec3 origin;
};

bool getSphereIntersection(vec3 dir, BoundingSphere s) {

// substitute into sphere equation
    float a = pow(dir.x, 2.0) +
              pow(dir.y, 2.0) +
              pow(dir.z, 2.0);
    float b = 2.0 * (dir.x * (u_Eye.x - s.origin.x) +
              dir.y * (u_Eye.y - s.origin.y) +
              dir.z * (u_Eye.z - s.origin.z));
    float c = pow((u_Eye.x - s.origin.x), 2.0) +
              pow((u_Eye.y - s.origin.y), 2.0) +
              pow((u_Eye.z - s.origin.z), 2.0) -
              pow(s.radius, 2.0);

// check to see if discrim is less than 0
    float discrim = pow(b, 2.0) - (4.0 * a * c);
    if (discrim >= 0.0) {
        return true;
    }
    else {
        return false;
    }
}


float sceneSDF(vec3 p) {

  /**********LOADING RINGS AND CIRCLES*********/

  float circleRadius = .75;
  float loadingRingRadius = 9.0 + sin(u_Time / 100.0);
  float loadingRingRadius2 = 9.0 + sin((u_Time + 40.0) / 100.0);
  float loadingRingRadius3 = 9.0 + sin((u_Time + 60.0) / 100.0);

  vec3 ringPoint = (inverse(rotateX(90.0)) * vec4(p, 1.0)).xyz;
  float loadingRing1 = sdTorus(ringPoint + vec3(0, 0, 0.7), vec2(loadingRingRadius, 0.15));


  float theta = mod(-1.0 * u_Time * 5.0, 361.0);
  float loadingRingCircle1 = sdSphere(p + vec3(calcCircle(theta, loadingRingRadius), 0.0), circleRadius);

  float loadingRing2 = sdTorus(ringPoint + vec3(0, 0, 0.7), vec2(loadingRingRadius2 * 3.0 - 14.0, 0.15));
  float theta2 = mod((u_Time + 40.0) * 1.5, 360.0);
  float loadingRingCircle4 = sdSphere(p + vec3(calcCircle(theta2, loadingRingRadius2 * 3.0 - 14.0), 0.0), circleRadius);

  float loadingRing3 = sdTorus(ringPoint + vec3(0, 0, 0.7), vec2(loadingRingRadius3 * 6.0 - 36.0, 0.15));
  float theta3 = mod(-1.0 * (u_Time + 100.0) * 3.2, 360.0);
  float loadingRingCircle5 = sdSphere(p + vec3(calcCircle(theta3, loadingRingRadius3 * 6.0 - 36.0), 0.0), circleRadius);
  
  
 /*********ADDING EVERYTHING TOGETHER***********/

  float blendFactor = u_Stickiness;
  float ring1 = loadingRing1;
  ring1 = opSmoothUnion(ring1, loadingRingCircle1, blendFactor);

  float ring2 = unionSDF(ring1, loadingRing2);
  ring2 = opSmoothUnion(ring2, loadingRingCircle4, blendFactor);

  float ring3 = unionSDF(ring2, loadingRing3);
  ring3 = opSmoothUnion(ring3, loadingRingCircle5, blendFactor);

  return ring3;
}

float sceneSDFRobot(vec3 p) {
  // bob entire robot up and down
  float pY = p.y;
  p.y = p.y + sin(u_Time / (100.0 - u_Bounce)) * 0.5;

  // body + ears
  float leftEarDist = sdSphere(p + vec3(-3.0, -1.114, .5), 0.861);
  float rightEarDist = sdSphere(p + vec3(3.0, -1.114, .5), 0.861);
  float ellipseDist = sdEllipsoid(p, vec3(3.528, 2.417, 2.417));


  // eyes
  vec3 eyePoint = (inverse(rotateX(90.0)) * vec4(p, 1.0)).xyz;
  float leftEyeDist = sdCappedCylinder(eyePoint + vec3(-1.2, 0, 0.5), 
                                vec2(0.497, 3.0));
  float rightEyeDist = sdCappedCylinder(eyePoint + vec3(1.2, 0, 0.5), 
                                vec2(0.497, 3.0));
  
  // antenna
  vec3 antPoint = (inverse(rotateZ(20.0)) * vec4(p, 1.0)).xyz;
  antPoint = antPoint + vec3(-0.8, -2.8, 0);
  float ant1 = sdCappedCylinder(antPoint, vec2(0.16, 1.6));
  vec3 antPoint2 = (inverse(rotateZ(110.0)) * vec4(p, 1.0)).xyz;
  float ant2 = sdCappedCylinder(antPoint2 + vec3(-4.26, -0.2, 0), vec2(0.16, 1.1));
  float ant3 = sdSphere(p + vec3(2.3, -3.7, 0), 0.5);


  // mouth
  vec3 mouthPoint = (inverse(rotateX(90.0)) * vec4(p, 1.0)).xyz;
  float mouthDist = sdCappedCylinder(mouthPoint + vec3(0, 0, 0.4), 
                                     vec2(1.747, 4.0));
  float mouthDist2 = sdCappedCylinder(mouthPoint+ vec3(0, 0, 0.7), 
                                     vec2(1.747, 5.0));
  float mouthCutCube = sdBox(p, vec3(1.8, 0.8, 5.0));
  mouthDist = differenceSDF(mouthDist, mouthDist2);
  mouthDist = differenceSDF(mouthDist, mouthCutCube);

  vec3 mouthCirclePoint = (inverse(rotateX(90.0)) * vec4(p, 1.0)).xyz;
  float leftMouthDist = sdCappedCylinder(eyePoint + vec3(-1.1, 0, -0.85), 
                                vec2(0.137, 4.0));
  float rightMouthDist = sdCappedCylinder(eyePoint + vec3(1.1, 0, -0.85), 
                                vec2(0.137, 4.0));

  float mouthCutCube2 = sdBox(p, vec3(1.2, 1.8, 5.0));
  mouthDist = opSmoothUnion(mouthDist, leftMouthDist, 0.20);
  mouthDist = opSmoothUnion(mouthDist, rightMouthDist, 0.20);
  mouthDist = opSmoothIntersection(mouthDist, mouthCutCube2, 0.1);
  float mouthDist3 = sdCappedCylinder(mouthPoint + vec3(0, 0, -0.3), 
                                     vec2(1.31, 4.0));
  mouthDist = opSmoothIntersection(mouthDist, mouthDist3, 0.1);

  /*********ADDING EVERYTHING TOGETHER***********/
  float oneEar = opSmoothUnion(leftEarDist, ellipseDist, 0.25);
  float bothEars = opSmoothUnion(oneEar, rightEarDist, 0.25);
  float oneEye = differenceSDF(bothEars, leftEyeDist);
  float twoEyes = differenceSDF(oneEye, rightEyeDist);

  float antPart1 = unionSDF(twoEyes, ant1);
  float antPart2 = opSmoothUnion(ant1, ant2, 0.13);
  antPart2 = unionSDF(antPart2, ant3);
  antPart2 = unionSDF(twoEyes, antPart2);
  antPart2 = differenceSDF(antPart2, mouthDist);
  p.y = pY;
  antPart2 = unionSDF(antPart2, sceneSDF(p));
  return antPart2;
}

vec3 estimateNormal(vec3 p, bool robot) {
  float EPSILON = 0.001;

  if (robot) {
    return normalize(vec3(
        sceneSDFRobot(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDFRobot(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDFRobot(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDFRobot(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDFRobot(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDFRobot(vec3(p.x, p.y, p.z - EPSILON))
    ));
  }
  else {
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
  }
}

// returns color
vec3 rayMarch(vec3 dir) {


  BoundingSphere rings = BoundingSphere((9.0 + sin((u_Time + 60.0) / 100.0) + 0.5) * 6.0 - 36.0,
                                        vec3(0, 0, 0.0));
  BoundingSphere robot = BoundingSphere(6.3, vec3(0, 0.7, 0));

  float depth = 0.0; 
  int MAX_MARCHING_STEPS = 1000;
  float EPSILON = 0.00001;
  float MAX_TRACE_DISTANCE = 1000.0;


  if (getSphereIntersection(dir, rings)) {
    bool biggerSphere = true;

    if (getSphereIntersection(dir, robot)) {
        biggerSphere = false;
    }

    for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
      vec3 point = u_Eye + depth * dir;

      float dist = 1.0;
      if (!biggerSphere) {
        dist = sceneSDFRobot(point);
      }
      else {
        dist = sceneSDF(point);
      }

      // we are inside the sphere!
      if (dist < EPSILON) {

        // gold shading
        vec3 diffuseColor = vec3(221, 82, 22) / 255.;
        float diffuseTerm = dot(normalize(vec3(-1, -1, -1) * estimateNormal(point, !biggerSphere)), normalize(u_Ref - u_Eye));
        diffuseTerm = clamp(diffuseTerm, 0.0, 1.0);
        float ambientTerm = 0.5;
        float PI = 3.14159265358979323846;

        vec3 color = vec3(0.5 + 0.5 * cos(2. * PI * (1.0 * diffuseTerm + 0.00)),
                          0.5 + 0.5 * cos(2. * PI * (0.7 * diffuseTerm + 0.15)),
                          0.5 + 0.5 * cos(2. * PI * (0.4 * diffuseTerm + 0.20)));
        return color;
      }

      // keep going!
      depth += dist;

      // we went too far ... we should stop
      if (depth >= MAX_TRACE_DISTANCE) {
        return vec3(227, 244, 252) / 255.;
      }
    }
  }
  return vec3(227, 244, 252) / 255.;
}

void main() {

  vec3 dir = getRayDirection();
  out_Col = vec4(rayMarch(dir), 1.0);
}
