#version 300 es
precision highp float;

uniform vec3 u_Eye, u_Ref, u_Up;
uniform vec2 u_Dimensions;
uniform float u_Time;
uniform float u_Stickiness;
uniform float u_Bounce;
uniform vec2 u_Trans;
uniform float u_Map;


in vec2 fs_Pos;
out vec4 out_Col;

float noise( vec3 p , vec3 seed) {
  return fract(sin(dot(p + seed, vec3(987.654, 123.456, 531.975))) * 85734.3545);
}

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

vec3 getRayDirection() {

  float fovy = 45.0;
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

vec2 path(float t) {
  float a = 3.0 *sin(t * .1 + 1.5) / (1.0 + noise(vec3(0.0, 0.0, t), 
                                                vec3(0.0, 0.0,0.0))); 
  float b = sin(t*.2) / (4.0 + noise(vec3(0.0, 0.0, t), 
                                     vec3(0.0, 0.0,0.0)));
  return vec2(2.*a, a*b);
}


float g = 0.;
float sceneSDF(vec3 p) {

  if (u_Map == 0.0) {
      p += vec3(sin(p.z / (20.0)) * 3.0, sin(p.z / 20.0) * 3.0, 0.0);
      vec3 ringPoint = (inverse(rotateX(90.0)) * vec4(p, 1.0)).xyz;
      vec3 c = vec3(0.0, 7.0, 0.0);
      vec3 q = mod(ringPoint,c)-0.5*c;
      //q += vec3(0.0, 0.0, sin(p.x) + 1.0);
      //q += vec3(0.0, 0.0, noise(vec3(0, 0, u_Time / 20.0), vec3(0, 0, 0)));
      return sdTorus(q, vec2(4.0, 0.15));
  }
  else {
    p.xy -= path(p.z);
    float d = -length(p.xy) + 4.;// tunnel (inverted cylinder)
    g += .015 / (.01 + d * d);
    return d;
  }

}

vec3 estimateNormal(vec3 p) {
  float EPSILON = 0.001;
    return normalize(vec3(
        sceneSDF(vec3(p.x + EPSILON, p.y, p.z)) - sceneSDF(vec3(p.x - EPSILON, p.y, p.z)),
        sceneSDF(vec3(p.x, p.y + EPSILON, p.z)) - sceneSDF(vec3(p.x, p.y - EPSILON, p.z)),
        sceneSDF(vec3(p.x, p.y, p.z  + EPSILON)) - sceneSDF(vec3(p.x, p.y, p.z - EPSILON))
    ));
}

// returns color
vec3 rayMarch(vec3 dir) {
  float time = u_Time / 1.5;
  vec3 eye = u_Eye + vec3(u_Trans.x / 8.0, u_Trans.y / 8.0, time);
  //vec3 eye = u_Eye;
  //vec3 eye = u_Eye + vec3(u_Trans.x / 100.0, u_Trans.y / 100.0, 0);
  
  float depth = 0.0; 
  int MAX_MARCHING_STEPS = 1000;
  float EPSILON = 0.0001;
  float MAX_TRACE_DISTANCE = 1000.0;

  vec4 fragCoord = gl_FragCoord;

  // vignette
  vec2 centerCoords = vec2(u_Dimensions.x / 2.0,
                       u_Dimensions.y / 2.0);
  float maxDistance = sqrt(pow(centerCoords.x, 2.0) +
                           pow(centerCoords.y, 2.0));
  float shortX = fragCoord.x - centerCoords.x;
  float shortY = fragCoord.y - centerCoords.y;
  float currentDistance = pow(shortX, 2.0) / pow(u_Dimensions.x, 2.0) +
                          pow(shortY, 2.0) / pow(u_Dimensions.y, 2.0);

  float intensity = 3.8; // how intense the vignette is
  float vignette = currentDistance * intensity;
  float intensity2 = 5.3; // how intense the vignette is
  float vignette2 = currentDistance * intensity2;
  vec3 vignetteColor = mix(vec3(1.0), vec3(0, 0, 0), vignette);
  vec3 vignetteColor2 = mix(vec3(0.0), vec3(1.0, 0.9, 0.7), (1.0 - vignette2));


    // center crosshair
  if (fragCoord.x < centerCoords.x + 12.0 &&
      fragCoord.x > centerCoords.x - 12.0 &&
      fragCoord.y < centerCoords.y + 0.8 &&
      fragCoord.y > centerCoords.y - 0.8) {
    return vec3(1.0, 1.0, 1.0);
  }
  if (fragCoord.x < centerCoords.x + 0.8 &&
    fragCoord.x > centerCoords.x - 0.8 &&
    fragCoord.y < centerCoords.y + 12.0 &&
    fragCoord.y > centerCoords.y - 12.0) {
  return vec3(1.0, 1.0, 1.0);
  }


    // ***************************** MAP #1 ****************************************
  if (u_Map == 0.0) {
    // INTERSECTION TESTING
      if (sceneSDF(eye) < EPSILON) {

        return vec3(0, 0, 1);
      }
      for (int i = 0; i < MAX_MARCHING_STEPS; i++) {
        vec3 point = eye + depth * dir;

          float dist = sceneSDF(point);
          // we are inside the sphere!
          if (dist < EPSILON) {
            // distance fog
            float dist = point.z - time;
            const vec3 fogColor = vec3(0.0, 0.0,0.1);
            float fogFactor = 0.;
            fogFactor = (70. - dist)/(70.0 - 30.0);
            fogFactor = clamp( fogFactor, 0.0, 1.0 );
    
                     // gold shading
                vec3 diffuseColor = vec3(221, 82, 22) / 255.;
                float diffuseTerm = dot(normalize(vec3(-1, -1, -1) * estimateNormal(point)), normalize(u_Ref - u_Eye));
                diffuseTerm = clamp(diffuseTerm, 0.0, 1.0);
                float ambientTerm = 0.5;
                float PI = 3.14159265358979323846;
    
                // vec3 color = vec3(0.5 + 0.5 * cos(2. * PI * (1.0 * diffuseTerm + 0.00)),
                //                   0.5 + 0.5 * cos(2. * PI * (0.7 * diffuseTerm + 0.15)),
                //                   0.5 + 0.5 * cos(2. * PI * (0.4 * diffuseTerm + 0.20)));
                vec3 color = vec3(cos(u_Time / 180.0) + 1.5, 
                                  sin(u_Time / 200.0) + 1.5, 
                                  cos(u_Time / 220.0) + 1.5);
                color = mix(fogColor, color, fogFactor);
    
                return color * vignetteColor;
          }

        // keep going!
        depth += dist;

         // we went too far ... we should stop
         if (depth >= MAX_TRACE_DISTANCE) {
           return vec3(0, 0, 0);
         }

      }

  }

 // ******************* MAP #2 ****************************
  else {

      vec3 ro = vec3(u_Trans.x / 8.0, u_Trans.y / 8.0, -5. + time / 2.0);
      vec3 p = floor(ro) + .5; 
      vec3 mask; 
      vec3 drd = 1.0 / abs(dir);
      dir = sign(dir);
      vec3 side = drd * (dir * (p - ro) + .5);
    
      float t = 0., ri = 0.;
    
      // INTERSECTION
      if (sceneSDF(ro) < EPSILON) {
        return vec3(0, 0, 1);
      }
      for (float i = 0.0; i < 1.0; i+= .01) {
          float dist = sceneSDF(p);

          if (dist < EPSILON) {
          
              break;
          }
          mask = step(side, side.yzx) * step(side, side.zxy);
          // minimum value between x,y,z, output 0 or 1
    
          side += drd * mask;
          p += dir * mask;
      }
    
      t = length(p - ro);
  
      vec3 c = vec3(1) * length(mask * vec3(1., .5, .3));
      c = mix(vec3(.8, .2, .7), vec3(.2, .1, .2), c);
      c += g * .4;
      c.g += sin(u_Time)*.2 + sin(p.z*.5 - u_Time * .1);// red rings
      c = mix(c, vec3(.2, .1, .2), 1. - exp(-.001*t*t));// fog
      return c;
  }
}

void main() {

  vec3 dir = getRayDirection();
  //out_Col = vec4(0.5 * (dir + vec3(1.0, 1.0, 1.0)), 1.0);
  out_Col = vec4(rayMarch(dir), 1.0);
}
