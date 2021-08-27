export const Vert = `
#version 100
precision highp float;

attribute vec3 LocalPosition;
attribute vec3 LocalUv;
varying vec3 FragColour;
varying vec3 WorldPosition;
varying vec3 FragLocalPosition;
varying vec2 FragLocalUv;
varying float TriangleIndex;

varying vec3 WorldUp;	//	up of LocalToWorldTransform

uniform mat4 LocalToWorldTransform;
uniform mat4 WorldToCameraTransform;
uniform mat4 CameraProjectionTransform;

uniform vec3 WorldBoundsBottom;
uniform vec3 WorldBoundsTop;
uniform float BoundsRadius;

void main()
{
	//	expecting cube 0..1
	vec3 LocalPos = LocalPosition;
	//vec4 WorldPos = LocalToWorldTransform * vec4(LocalPos,1);
	vec4 WorldPos;
	WorldPos.xyz = mix( WorldBoundsBottom, WorldBoundsTop, LocalPos.y );
	WorldPos.x += mix( -BoundsRadius, BoundsRadius, LocalPos.x );
	WorldPos.z += mix( -BoundsRadius, BoundsRadius, LocalPos.z );
	WorldPos.y += mix( -BoundsRadius, BoundsRadius, LocalPos.y );
	WorldPos.w = 1.0;
	
	vec4 CameraPos = WorldToCameraTransform * WorldPos;	//	world to camera space
	vec4 ProjectionPos = CameraProjectionTransform * CameraPos;
	gl_Position = ProjectionPos;
	
	WorldPosition = WorldPos.xyz;
	FragColour = vec3( LocalUv );
	FragLocalPosition = LocalPosition;
	FragLocalUv = LocalUv.xy;
	TriangleIndex = LocalUv.z;
	
	vec4 WorldUp4 = LocalToWorldTransform * vec4(0,1,0,0);
	WorldUp = WorldUp4.xyz;//normalize(WorldUp4.xyz / WorldUp4.w);
}



`;

/*
export const Frag = `
#version 100
precision highp float;
varying vec2 FragLocalUv;
void main()
{
	gl_FragColor = vec4( FragLocalUv, 1.0, 1 );
}
`;
*/
export const Frag = `
precision highp float;

varying vec3 WorldPosition;
uniform vec3 WorldBoundsBottom;
uniform vec3 WorldBoundsTop;
uniform float BoundsRadius;
uniform float DrinkRadius;

uniform mat4 ScreenToCameraTransform;
uniform mat4 CameraToWorldTransform;
const vec4 Sphere = vec4(0,0,0,0.5);
const bool DrawNormals = false;
const bool DrawShadows = true;
const bool DrawHeat = false;

uniform float TimeNormal;

const vec3 WorldUp = vec3(0,-1,0);
const float FloorY = 2.0;
#define FAR_Z		20.0
#define MAX_STEPS	30
#define CLOSEENOUGH_FOR_HIT	0.001
float Range(float Min,float Max,float Value)
{
	return (Value-Min) / (Max-Min);
}
vec3 Range3(vec3 Min,vec3 Max,vec3 Value)
{
	Value.x = Range( Min.x, Max.x, Value.x );
	Value.y = Range( Min.y, Max.y, Value.y );
	Value.z = Range( Min.z, Max.z, Value.z );
	return Value;
}
float Range01(float Min,float Max,float Value)
{
	return clamp( Range(Min,Max,Value), 0.0, 1.0 );
}
vec3 ScreenToWorld(vec2 uv,float z)
{
	//float x = mix( -1.0, 1.0, uv.x );
	//float y = mix( 1.0, -1.0, uv.y );
	float x = mix( -1.0, 1.0, uv.x );
	float y = mix( -1.0, 1.0, uv.y );
	vec4 ScreenPos4 = vec4( x, y, z, 1.0 );
	vec4 CameraPos4 = ScreenToCameraTransform * ScreenPos4;
	vec4 WorldPos4 = CameraToWorldTransform * CameraPos4;
	vec3 WorldPos = WorldPos4.xyz / WorldPos4.w;
	
	return WorldPos;
}
//	gr: returning a TRay, or using TRay as an out causes a very low-precision result...
void GetWorldRay(out vec3 RayPos,out vec3 RayDir)
{
	float Near = 0.01;
	float Far = 10.0;
	
	//	ray goes from camera
	//	to WorldPosition, which is the triangle's surface pos
	vec4 CameraWorldPos4 = CameraToWorldTransform * vec4(0,0,0,1);
	vec3 CameraWorldPos3 = CameraWorldPos4.xyz / CameraWorldPos4.w;
	RayPos = CameraWorldPos3;
	
	RayDir = WorldPosition - RayPos;
	
	RayDir = normalize(RayDir);
	//	gr: this is backwards!
	//RayDir = -normalize( RayDir );
}
float PingPongNormal(float Normal)
{
	//	0..1 to 0..1..0
	if ( Normal >= 0.5 )
	{
		Normal = Range( 1.0, 0.5, Normal );
	}
	else
	{
		Normal = Range( 0.0, 0.5, Normal );
	}
	return Normal;
}
float sdBox( vec3 p, vec3 b )
{
	vec3 q = abs(p) - b;
	return length(max(q,0.0)) + min(max(q.x,max(q.y,q.z)),0.0);
}
float DistanceToBox(vec3 Position,vec3 BoxCenter,vec3 BoxRadius)
{
	return sdBox( Position-BoxCenter, BoxRadius );
}
float rand(vec3 co)
{
	return fract(sin(dot(co, vec3(12.9898, 78.233, 54.53))) * 43758.5453);
}
vec3 Repeat(vec3 Position,float c)
{
	//	move to center of 0..c before modulous
	Position += c * 0.5;
	//vec3 Grid3 = floor( Position / c );
	vec3 Grid3 = floor( Position / c );
	Position = mod( Position, c );
		
	Position -= c * 0.5;
	
	//	position randomly inside the repeat cube
	//	based on which cube we're in
	float Randf = mix( -0.5, 0.5, rand(Grid3) );
	Randf *= 0.4;
	Position += c * vec3(Randf,Randf,Randf);
	
	
	
	return Position;
}
vec3 BendShape(vec3 Position, float k)
{
	//	bend shape
	vec3 p = Position;
	//float k = 2.699;//TimeSecs*0.1; // or some other amount
	float c = cos(k*p.x);
	float s = sin(k*p.x);
	mat2  m = mat2(c,-s,s,c);
	vec3  q = vec3(m*p.xy,p.z);
	return q;
}

float dot2( in vec2 v ) { return dot(v,v); }
float dot2( in vec3 v ) { return dot(v,v); }
float ndot( in vec2 a, in vec2 b ) { return a.x*b.x - a.y*b.y; }

float sdCappedCone( vec3 p, float h, float r1, float r2 )
{
	vec2 q = vec2( length(p.xz), p.y );
	vec2 k1 = vec2(r2,h);
	vec2 k2 = vec2(r2-r1,2.0*h);
	vec2 ca = vec2(q.x-min(q.x,(q.y<0.0)?r1:r2), abs(q.y)-h);
	vec2 cb = q - k1 + k2*clamp( dot(k1-q,k2)/dot2(k2), 0.0, 1.0 );
	float s = (cb.x<0.0 && ca.y<0.0) ? -1.0 : 1.0;
	return s*sqrt( min(dot2(ca),dot2(cb)) );
}

float sdCappedCylinder( vec3 p, float h, float r )
{
	//	gr: height and radius were backwards??
	//vec2 d = abs(vec2(length(p.xz),p.y)) - vec2(h,r);
	vec2 d = abs(vec2(length(p.xz),p.y)) - vec2(r,h);
	return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float DistanceToCappedCylinder_StartEnd(vec3 Position,vec3 Bottom,vec3 Top,float Radius)
{
	vec3 Center = mix( Bottom, Top, 0.5 );
	float Height = length( Bottom - Top ) / 2.0;
	Position -= Center;
	return sdCappedCylinder( Position, Height, Radius ); 
}

 

float DistanceToGlass(vec3 Position)
{
	vec3 GlassCenter = mix( WorldBoundsBottom, WorldBoundsTop, 0.5 );
	float GlassHeight = length( WorldBoundsBottom - WorldBoundsTop ) / 2.0;
	
	float Rounding = 0.0;//GlassHeight * 0.2;
	Rounding = min( Rounding, 0.005 );
	
	
	GlassHeight -= Rounding * 2.0;
	float GlassRadius = DrinkRadius - Rounding - Rounding;
	float TopRadius = GlassRadius;
	float BottomRadius = GlassRadius * 0.7;
	float Distance = sdCappedCone( Position - GlassCenter, GlassHeight, BottomRadius, TopRadius );
	Distance -= Rounding;
	return Distance;
}

const float BottomHeightPercent = 0.3;
const float TopHeightPercent = 0.5;
const float FlyHeight = 0.3;

float DistanceToLiquidBottom(vec3 Position)
{
	vec3 Bottom = WorldBoundsBottom;
	vec3 BottomChunk = (WorldBoundsTop - WorldBoundsBottom) * BottomHeightPercent;
	vec3 Top = Bottom + BottomChunk;
	float Distance = DistanceToCappedCylinder_StartEnd( Position, Bottom, Top, DrinkRadius );
	
	//	smooth edges
	Distance -= 0.01;
	return Distance;
}


float GetDisplacement(vec3 PositionSeed,float Frequency,float Scale)
{
	PositionSeed *= vec3(Frequency,Frequency,Frequency);
	float Displacement = 1.0;
	//Displacement *= sin(PositionSeed.x);
	//Displacement *= sin(PositionSeed.y);
	//Displacement *= sin(PositionSeed.z);
	Displacement *= Scale;
	return Displacement;
}


float opSmoothUnion( float d1, float d2, float k ) {
    float h = clamp( 0.5 + 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) - k*h*(1.0-h); }
float opSmoothIntersection( float d1, float d2, float k ) {
    float h = clamp( 0.5 - 0.5*(d2-d1)/k, 0.0, 1.0 );
    return mix( d2, d1, h ) + k*h*(1.0-h); }
    
float sdPlane( vec3 p, vec3 n, float h )
{
	// n must be normalized
	return dot(p,n) + h;
}
float DistanceToFloor(vec3 Position)
{
	float Distance = sdPlane( Position, WorldUp, FloorY );
	return Distance;
}

float DistanceToLiquidTop(vec3 Position)
{
	vec3 TimeOffset = vec3(0,TimeNormal * FlyHeight,0);
	vec3 BottomChunk = (WorldBoundsTop - WorldBoundsBottom) * BottomHeightPercent;
	vec3 BottomTop = WorldBoundsBottom + BottomChunk;
	BottomTop += TimeOffset;
	
	vec3 Chunk = (WorldBoundsTop - WorldBoundsBottom) * (TopHeightPercent);
	vec3 Top = BottomTop + Chunk;
	
	
	float Distance = DistanceToCappedCylinder_StartEnd( Position, BottomTop, Top, DrinkRadius );
	
	//	smooth edges
	Distance -= 0.01;
	
	//	wobble
	float DistanceWobbled = TimeNormal * GetDisplacement( Position /*- TimeOffset*/, 24.0 + TimeNormal, 0.2*TimeNormal );
	DistanceWobbled += Distance;

	float Smoothk = 0.30 * TimeNormal;
	float SmoothedDistance = opSmoothUnion( DistanceWobbled, Distance, Smoothk ); 

	return SmoothedDistance;
}

float opIntersection( float d1, float d2 ) 
{
	return max(d1,d2);
}

float opUnion( float d1, float d2 ) 
{
	return min(d1,d2); 
}

float DistanceToSphere(vec3 Position,vec3 Center,float Radius)
{
	float Distance = length( Center - Position );
	Distance -= Radius;
	return Distance;
}

float DistanceToLiquidBounds(vec3 Position)
{
	//	glass bounds
	float Glass = DistanceToCappedCylinder_StartEnd( Position, WorldBoundsBottom, WorldBoundsTop, DrinkRadius );
	
	//	some space on top
	float RadiusAbove = 0.60;
	float DistanceAbove = 0.59;//	should be able to calc this to work out where edge of sphere hits edge of rim
	float Above = DistanceToSphere( Position, WorldBoundsTop+vec3(0,DistanceAbove,0), RadiusAbove );
	//float Above = DistanceToCappedCylinder_StartEnd( Position, WorldBoundsTop, WorldBoundsTop+vec3(0,1,0), DrinkRadius*10.0 );
	
	return opUnion( Glass, Above );
}

#define LiquidSphereDim	3
#define LiquidSpherePositonCount	(LiquidSphereDim*LiquidSphereDim*LiquidSphereDim)
uniform vec4 LiquidSpherePositons[LiquidSpherePositonCount];

float DistanceToLiquidSpheres(vec3 Position)
{
	float RadiusScaleMin = 0.009;
	float RadiusScaleMax = 0.002;
	float PositionScale = 1.00;
	float Distance = 999.0;

	for ( int i=0;	i<LiquidSpherePositonCount;	i++ )
	{
		//	todo: put input into local space, instead of scaling liquid pos 
		vec3 LiquidPos = LiquidSpherePositons[i].xyz * vec3(PositionScale,PositionScale,PositionScale);
		
		float LiquidRadius = LiquidSpherePositons[i].w;
		LiquidRadius = mix( RadiusScaleMin, RadiusScaleMax, LiquidRadius );

		//	dont let liquid go below surface
		LiquidPos.y = max( LiquidPos.y-LiquidRadius, WorldBoundsBottom.y );

		LiquidPos -= Position;
		float NewDistance = length( LiquidPos );
		NewDistance -= LiquidRadius;
		
		float Smoothk = 0.04;
		Distance = opSmoothUnion(Distance,NewDistance,Smoothk);
		//Distance = min( Distance, NewDistance );
	}

	//	dont let liquid go below surface
	float FloorDistance = sdPlane( Position, WorldUp, WorldBoundsBottom.y );
	Distance = max( Distance, FloorDistance );

	return Distance;
}


float DistanceToLiquid(vec3 Position)
{
	return DistanceToLiquidSpheres(Position);
	float Bottom = DistanceToLiquidBottom(Position);
	float Top = DistanceToLiquidTop(Position);
	//	blend together
	float Smoothk = 0.0015;
	float LiquidDistance = opSmoothUnion(Bottom,Top,Smoothk);
	
	//	strict intersection(AND) with glass shape
	float LiquidBounds = DistanceToLiquidBounds(Position);
	
	float Distance = opIntersection( LiquidBounds, LiquidDistance );
	return Distance;
}

float DistanceToSphere(vec3 Position)
{
/*
	Position = Repeat( Position, Sphere.w * 10.0 );
	
	//Position = BendShape( Position, 1.0);//2.699 );
	return DistanceToBox( Position, Sphere.xyz, Sphere.www );
	*/
	//float SphereRadius = PingPongNormal(fract(TimeSecs)) * Sphere.w;
	float SphereRadius = DrinkRadius;
	float Distance = length(WorldBoundsBottom.xyz - Position);
	Distance -= SphereRadius;
	return Distance;
}

float map(vec3 Position)
{
	float GlassDistance = DistanceToGlass( Position );
	float LiquidDistance = DistanceToLiquid( Position );
	
	return opSmoothUnion( GlassDistance, LiquidDistance, 0.03 );
	
	//float FloorDistance = DistanceToFloor( Position );
	return min( GlassDistance, LiquidDistance );
	return LiquidDistance;
}
//	xyz heat (0= toofar/miss)
vec4 GetSceneIntersection(vec3 RayPos,vec3 RayDir)
{
	RayDir = normalize(RayDir);
	const float CloseEnough = CLOSEENOUGH_FOR_HIT;
	const float MinStep = CloseEnough;
	const float MaxDistance = FAR_Z;
	const int MaxSteps = MAX_STEPS;
	
	//return vec4( RayPos, 1.0 );
	//return vec4( RayPos + RayDir * 1.0, 1.0 );
	
	//	time = distance
	float RayTime = 0.0;
	
	for ( int s=0;	s<MaxSteps;	s++ )
	{
		vec3 Position = RayPos + RayDir * RayTime;
		
		//	intersect scene
		float HitDistance = map( Position );
		if ( HitDistance <= CloseEnough )
		{
			float Heat = 1.0 - (float(s)/float(MaxSteps));
			return vec4( Position, Heat );
		}
		RayTime += max( HitDistance, MinStep );
		
		//	ray gone too far
		if (RayTime > MaxDistance)
			return vec4(Position,0);
	}
	
	return vec4(0,0,0,0);
}
vec3 calcNormal( in vec3 pos )
{
	vec2 e = vec2(1.0,-1.0)*0.5773;
	const float eps = 0.0005;
	return normalize( e.xyy * map( pos + e.xyy*eps ) + 
					  e.yyx * map( pos + e.yyx*eps ) + 
					  e.yxy * map( pos + e.yxy*eps ) + 
					  e.xxx * map( pos + e.xxx*eps ) );
}
float HeatToShadow(float Heat)
{
	return Heat > 0.0 ? 1.0 : 0.0;
	return clamp( Range( 0.0, 0.5, Heat ), 0.0, 1.0 );
}
void main()
{
	vec3 Background = vec3(0.70,0.75,0.79);
	gl_FragColor = vec4(Background,1.0);

	vec3 RayPos,RayDir;
	GetWorldRay(RayPos,RayDir);
	vec4 Intersection = GetSceneIntersection( RayPos, RayDir );
	if ( Intersection.w <= 0.0 )
	{
		gl_FragColor = vec4(Background,0.0);
		discard;
		return;
	}
	
	vec3 Colour;
	vec3 Normal = calcNormal(Intersection.xyz);
	{
		Colour = Range3( vec3(-1,-1,-1), vec3(1,1,1), Normal );
	}
	Colour[2] = TimeNormal;

	//Colour = mix( Background, Colour, Intersection.w );
	if ( DrawNormals )
	{
		vec3 Normal = calcNormal(Intersection.xyz);
		Normal = Range3( vec3(-1,-1,-1), vec3(1,1,1), Normal );
		gl_FragColor = vec4( Normal,1.0);
		return;
	}
	/*
	if ( DrawHeat )
	{
		float Shadow = HeatToShadow( Intersection.w );
		gl_FragColor = vec4( Shadow, Shadow, Shadow, 1.0);
		return;
	}
	
	//	do a hard shadow pass by shooting a ray to the sun
	if ( DrawShadows )
	{
		//vec3 DirToLight = vec3(0.001,-0.99,0.001);
		vec3 LightPos = vec3( sin(TimeSecs)*20.0, 50.0, cos(TimeSecs)*20.0 );
		vec3 DirToLight = normalize(Intersection.xyz - LightPos);
		//vec3 PositionToLight = Intersection.xyz+(Normal*0.002);
		vec3 PositionToLight = Intersection.xyz+(DirToLight*0.001);
		vec4 LightIntersection = GetSceneIntersection( PositionToLight, DirToLight );
		if ( LightIntersection.w > 0.0 )
		{
			//float Shadow = HeatToShadow( LightIntersection.w );
			//float Shadow = 1.0;
			float Shadow = Range01( 50.0, 0.0, length( LightIntersection.xyz - Intersection.xyz )+10.0 );
			Shadow *= LightIntersection.w;
			Shadow = 0.9;
			Colour = mix( Colour, vec3(0,0,0), Shadow );
			//Colour = vec3(0,0,0);
		}
	}
*/
	
	{
		gl_FragColor = vec4(Colour,1);
	}

}
`;

export default Vert;
