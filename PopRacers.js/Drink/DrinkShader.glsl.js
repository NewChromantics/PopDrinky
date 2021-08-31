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

#define RENDER_SDFMAPS	16

#if defined(RENDER_SDFMAPS)

uniform sampler2D SdfMap_0;
uniform sampler2D SdfMap_1;
uniform sampler2D SdfMap_2;
uniform sampler2D SdfMap_3;
uniform sampler2D SdfMap_4;
uniform sampler2D SdfMap_5;
uniform sampler2D SdfMap_6;
uniform sampler2D SdfMap_7;
uniform sampler2D SdfMap_8;
uniform sampler2D SdfMap_9;
uniform sampler2D SdfMap_10;
uniform sampler2D SdfMap_11;
uniform sampler2D SdfMap_12;
uniform sampler2D SdfMap_13;
uniform sampler2D SdfMap_14;
uniform sampler2D SdfMap_15;
uniform vec3 SdfMapMin[RENDER_SDFMAPS];
uniform vec3 SdfMapMax[RENDER_SDFMAPS];
#define SdfMapMinFirst	SdfMapMin[0]
#define SdfMapMaxFirst	SdfMapMax[0]
#define SdfMapMinLast	SdfMapMin[RENDER_SDFMAPS-1]
#define SdfMapMaxLast	SdfMapMax[RENDER_SDFMAPS-1]
#else
#endif
uniform vec3 WorldBoundsBottom;
uniform vec3 WorldBoundsTop;
uniform float BoundsRadius;
uniform float DrinkRadius;

//#endif

varying vec3 WorldPosition;
uniform mat4 ScreenToCameraTransform;
uniform mat4 CameraToWorldTransform;
const vec4 Sphere = vec4(0,0,0,0.5);
const bool DrawNormals = false;
const bool DrawShadows = true;
const bool DrawHeat = false;

uniform float TimeNormal;
uniform float RealTime;

#define Material_None		0.0
#define Material_Liquid		1.0
#define Material_Floor		2.0
#define Material_BrownSugar	3.0
#define IntersectionMat_t	vec4
#define DistanceMaterial_t	vec2

//#define FLOOR_PLANE_VISIBLE
#if defined(FLOOR_PLANE_VISIBLE)
const vec4 FloorColour = vec4(1,1,0,1);
#else
const vec4 FloorColour = vec4(0,0,0,0);
#endif

const vec3 BrownSugarColour = vec3(110, 64, 8)/vec3(255,255,255);
const vec3 LightPos = vec3( 0.6, 2.0, 1.2 );

const vec3 WorldUp = vec3(0,-1,0);
const float FloorY = 0.1;
#define FAR_Z		20.0
#define MAX_STEPS	20
#define CLOSEENOUGH_FOR_HIT	0.001

const float BottomHeightPercent = 0.3;
const float TopHeightPercent = 0.5;
const float FlyHeight = 0.3;
#define LiquidSpherePositionCount	25
uniform vec4 LiquidSpherePositions[LiquidSpherePositionCount];

#define IngredientPositionCount	2
uniform vec4 IngredientPositions[IngredientPositionCount];


vec3 NormalToRedGreen(float Normal)
{
	if ( Normal < 0.0 )
	{
		return vec3( 1,0,1 );
	}
	else if ( Normal < 0.5 )
	{
		Normal = Normal / 0.5;
		return vec3( 1, Normal, 0 );
	}
	else if ( Normal <= 1.0 )
	{
		Normal = (Normal-0.5) / 0.5;
		return vec3( 1.0-Normal, 1, 0 );
	}
	
	//	>1
	return vec3( 0,0,1 );
}

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

float DistanceToLiquidBottom(vec3 Position)
{
	vec3 Bottom = WorldBoundsBottom;
	float BottomHeightPercent = 1.0 - TimeNormal;
	BottomHeightPercent = max( BottomHeightPercent, 0.3 );
	vec3 BottomChunk = (WorldBoundsTop - WorldBoundsBottom) * BottomHeightPercent;
	vec3 Top = Bottom + BottomChunk;
	float Distance = DistanceToCappedCylinder_StartEnd( Position, Bottom, Top, DrinkRadius );
	
	//	smooth edges
	//Distance -= 0.01;
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
	float Distance = sdPlane( Position, vec3(0,1,0), WorldBoundsBottom.y );
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

mat4 rotationX( in float angle ) {
	return mat4(	1.0,		0,			0,			0,
			 		0, 	cos(angle),	-sin(angle),		0,
					0, 	sin(angle),	 cos(angle),		0,
					0, 			0,			  0, 		1);
}

mat4 rotationY( in float angle ) {
	return mat4(	cos(angle),		0,		sin(angle),	0,
			 				0,		1.0,			 0,	0,
					-sin(angle),	0,		cos(angle),	0,
							0, 		0,				0,	1);
}

mat4 rotationZ( in float angle ) {
	return mat4(	cos(angle),		-sin(angle),	0,	0,
			 		sin(angle),		cos(angle),		0,	0,
							0,				0,		1,	0,
							0,				0,		0,	1);
}

float DistanceToIngredients(vec3 Position)
{
	float CubeRadiusMin = 0.015;
	//CubeRadiusMin *= TimeNormal;//	just aids hiding
	float CubeRadiusMax = CubeRadiusMin;
	
	float Distance = 999.0;

	for ( int i=0;	i<IngredientPositionCount;	i++ )
	{
		vec3 IngredientPos = IngredientPositions[i].xyz;

		float GridCount = 14.0; 
		
		float CubeRadius = IngredientPositions[i].w;
		CubeRadius = mix( CubeRadiusMin, CubeRadiusMax, CubeRadius );
		vec3 CubeLocalPos = vec3(0,0,0);
		//float CubeLocalRadius = CubeRadius / GridCount;
		float CubeLocalRadius = CubeRadius;
	
	
	
		//	find the nearest local cube
		vec3 LocalPos = Position - IngredientPos;
		LocalPos /= CubeRadius * 2.0;
		LocalPos = clamp( LocalPos, vec3(-1,-1,-1), vec3(1,1,1) );
/*
vec3 GriddedLocalPos = floor(LocalPos * GridCount)/GridCount;
float Random = rand(GriddedLocalPos);
//CubeLocalRadius += Random * 0.01;
//LocalPos.y += Random * 0.11;
LocalPos -= GriddedLocalPos + (0.5 /GridCount);
LocalPos = (rotationY(TimeNormal) * vec4(LocalPos,0.0)).xyz;
LocalPos = (rotationX(Random) * vec4(LocalPos,0.0)).xyz;
LocalPos += GriddedLocalPos  + (0.5 /GridCount);
		
		//	back to world space size
		LocalPos *= CubeRadius * 2.0;

		CubeLocalPos = LocalPos;

		//	turn into small cubes
		//	find the closest in local space
		
		
		//	back to world pos
		//IngredientPos = LocalPos * CubeRadius * 2.0;
/*
		//	turn into small cubes
		vec3 LocalPos = Position - IngredientPos;
		LocalPos /= CubeRadius;
		float GridCount = 4.0; 
		LocalPos = floor(LocalPos * GridCount)/GridCount;
		float Random = rand(LocalPos);
		CubeRadius *= Random;
	*/
		vec3 CubePos = IngredientPos + CubeLocalPos;
		float NewDistance = DistanceToBox( Position, CubePos, vec3(CubeLocalRadius,CubeLocalRadius,CubeLocalRadius) );

/*
		//	distort in grid
		//	not awful... has potential
		float GridCount = 10.0; 
		LocalPos = floor(LocalPos * GridCount)/GridCount;
		float Random = rand(LocalPos);
		//float Random = LocalPos.x;
		NewDistance += Random * 0.01;
	*/
		//float GridCount = 90.0; 
		GridCount = 50.0; 
		LocalPos = floor(LocalPos * GridCount)/GridCount;
		float Random = rand(LocalPos);
		//float Random = LocalPos.x;
		NewDistance += Random * 0.0009;

		
		Distance = min( Distance, NewDistance );
	}

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


float DistanceToLiquidSpheres(vec3 Position)
{
	float RadiusScaleMin = 0.002;
	float RadiusScaleMax = 0.009;
	float PositionScale = 1.00;
	float Distance = 999.0;
	float SmoothkMin = 0.03;
	float SmoothkMax = 0.05;
	float Smoothk = mix( SmoothkMin, SmoothkMax, 1.0-TimeNormal );

	for ( int i=0;	i<LiquidSpherePositionCount;	i++ )
	{
		//	todo: put input into local space, instead of scaling liquid pos 
		vec3 LiquidPos = LiquidSpherePositions[i].xyz * vec3(PositionScale,PositionScale,PositionScale);
		
		float LiquidRadius = LiquidSpherePositions[i].w;
		LiquidRadius = mix( RadiusScaleMin, RadiusScaleMax, LiquidRadius );

		//	dont let liquid go below surface
		LiquidPos.y = max( LiquidPos.y-LiquidRadius, WorldBoundsBottom.y );

		//	this is sphere distance
		LiquidPos -= Position;
		float NewDistance = length( LiquidPos );
		NewDistance -= LiquidRadius;
		
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
	float LiquidDistance = DistanceToLiquidSpheres(Position);
	
	float Bottom = DistanceToLiquidBottom(Position);
	float Smoothk = 0.04;
	//LiquidDistance = opSmoothUnion(Bottom,Top,Smoothk);
	LiquidDistance = opSmoothUnion(LiquidDistance,Bottom,Smoothk);
	//LiquidDistance = opUnion(LiquidDistance,Bottom);


	/*
	float Top = DistanceToLiquidTop(Position);
	//	blend together
	float Smoothk = 0.0015;
	float LiquidDistance = opSmoothUnion(Bottom,Top,Smoothk);
	LiquidDistance = opSmoothUnion(LiquidDistance,Liquid,Smoothk);
	*/
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

DistanceMaterial_t NearestMaterial(DistanceMaterial_t a,DistanceMaterial_t b)
{
	return (a.x < b.x) ? a : b;
}

#if defined(RENDER_SDFMAPS)
DistanceMaterial_t SampleSdfMapX(vec3 Position,sampler2D Map)
{
	float u = Range( SdfMapMinFirst.x, SdfMapMaxFirst.x, Position.x );
	float v = Range( SdfMapMinFirst.y, SdfMapMaxFirst.y, Position.y );
	vec4 Sample = texture2D(Map, vec2(u,v) );
	return Sample.xz;
}
DistanceMaterial_t SampleSdfMap(vec3 Position,int Index)
{
	if ( Index == 0 )	return SampleSdfMapX( Position, SdfMap_0 );	
	if ( Index == 1 )	return SampleSdfMapX( Position, SdfMap_1 );	
	if ( Index == 2 )	return SampleSdfMapX( Position, SdfMap_2 );	
	if ( Index == 3 )	return SampleSdfMapX( Position, SdfMap_3 );	
	if ( Index == 4 )	return SampleSdfMapX( Position, SdfMap_4 );	
	if ( Index == 5 )	return SampleSdfMapX( Position, SdfMap_5 );	
	if ( Index == 6 )	return SampleSdfMapX( Position, SdfMap_6 );	
	if ( Index == 7 )	return SampleSdfMapX( Position, SdfMap_7 );	
	if ( Index == 8 )	return SampleSdfMapX( Position, SdfMap_8 );	
	if ( Index == 9 )	return SampleSdfMapX( Position, SdfMap_9 );	
	if ( Index == 10 )	return SampleSdfMapX( Position, SdfMap_10 );	
	if ( Index == 11 )	return SampleSdfMapX( Position, SdfMap_11 );	
	if ( Index == 12 )	return SampleSdfMapX( Position, SdfMap_12 );	
	if ( Index == 13 )	return SampleSdfMapX( Position, SdfMap_13 );	
	if ( Index == 14 )	return SampleSdfMapX( Position, SdfMap_14 );	
	if ( Index == 15 )	return SampleSdfMapX( Position, SdfMap_15 );	
	/*	
	if ( Index == 16 )	return SampleSdfMapX( Position, SdfMap_16 );
	if ( Index == 17 )	return SampleSdfMapX( Position, SdfMap_17 );	
	if ( Index == 18 )	return SampleSdfMapX( Position, SdfMap_18 );	
	if ( Index == 19 )	return SampleSdfMapX( Position, SdfMap_19 );	
	return SampleSdfMapX( Position, SdfMap_19 );
	*/
	return SampleSdfMapX( Position, SdfMap_15 );
}
DistanceMaterial_t MapSdfMaps(vec3 Position)
{
	//	work out which slice index (z) to use
	float Inside = 1.0;
	float zf = Range( SdfMapMinFirst.z, SdfMapMaxLast.z, Position.z );
	
	zf *= float(RENDER_SDFMAPS);
	int zi = int(floor(zf));
	zf = zf - floor(zf);

#define MIN_SLICE	0
#define MAX_SLICE	(RENDER_SDFMAPS-1)
//#define MIN_SLICE	2
//#define MAX_SLICE	9
	if ( zi < MIN_SLICE )	
	{
		zi = MIN_SLICE;
		Inside = 0.0;
	}
	else if ( zi > MAX_SLICE-1 )	
	{
		zi = MAX_SLICE-1;
		Inside = 0.0;
	}
	
	
	//zi = int( mix( 0.0, float(RENDER_SDFMAPS-2), RealTime ) );
	//zi = int( mix( 0.0, float(RENDER_SDFMAPS-2), TimeNormal ) );
	//zi = 2;
	DistanceMaterial_t Start = SampleSdfMap(Position,zi+0);
	DistanceMaterial_t End = SampleSdfMap(Position,zi+1);
	
	DistanceMaterial_t Result = mix( Start, End, zf );
	//	force 1 material
	Result.y = Start.y;
	
	
	//	not sure how neccessary this is? should clip at edges
	float u = Range( SdfMapMinFirst.x, SdfMapMaxFirst.x, Position.x );
	float v = Range( SdfMapMinFirst.y, SdfMapMaxFirst.y, Position.y );
	if ( u < 0.0 || u > 1.0 || v < 0.0 || v>1.0 )
		Inside = 0.0;

	
	//	gr: I think we're geting a repeat when on far side
	
	//	outside slices, return distance to the block of slices
	bool RenderBounds = false;
	if ( Inside < 0.5 || RenderBounds )
	{
		vec3 SdfMapMin_i = SdfMapMin[0];
		vec3 SdfMapMax_i = SdfMapMax[0];
		//	gr: I dont think this clamped pos is neccessarily the "nearest" point, but seems to work...
		vec3 SlicePosition;
		SlicePosition.x = clamp( Position.x, SdfMapMin_i.x, SdfMapMax_i.x );
		SlicePosition.y = clamp( Position.y, SdfMapMax_i.y, SdfMapMin_i.y );//	reversed!
		SlicePosition.z = clamp( Position.z, SdfMapMinFirst.z, SdfMapMaxLast.z );
		float StepInside = RenderBounds ? 0.00 : 0.01;
		//	does "correct" distance to the slice range
		Result.x = length( Position - SlicePosition ) + StepInside;
	}
	
	return Result;
}
#endif

DistanceMaterial_t map(vec3 Position)
{
#if defined(RENDER_SDFMAPS)
	return MapSdfMaps( Position );
#else

	//float GlassDistance = DistanceToGlass( Position );
	float LiquidDistance = DistanceToLiquid( Position );
	//float Distance = opSmoothUnion( GlassDistance, LiquidDistance, 0.03 );
	float Distance = LiquidDistance;
	DistanceMaterial_t Liquid = DistanceMaterial_t( Distance, Material_Liquid );
	//return Liquid;
	
	DistanceMaterial_t Ingredient = DistanceMaterial_t( DistanceToIngredients(Position), Material_BrownSugar );
	//return Ingredient;
	
	//DistanceMaterial_t Floor = vec2( DistanceToFloor( Position ), Material_Floor );
	//return NearestMaterial( Liquid, Floor );
	return NearestMaterial( Liquid, Ingredient );
	return Liquid;
#endif
}


//	w = material
vec4 GetSceneIntersection(vec3 RayPos,vec3 RayDir)
{
	RayDir = normalize(RayDir);
	const float CloseEnough = CLOSEENOUGH_FOR_HIT;
	const float MinStep = CloseEnough;
	const int MaxSteps = MAX_STEPS;
	const float StepBack = 0.0;

	//	time = distance
	float RayTime = 0.0;
	float MaxDistance = FAR_Z;

	vec4 MarchMissHit = vec4(0,0,0,Material_None);

	//	ray trace to floor, and set our fail intersection to the floor
	//	we can also cut back max distance
	DistanceMaterial_t FloorHit = vec2( DistanceToFloor( RayPos ), Material_Floor );
	{
		// raytrace floor plane
		//float tp1 = dot(WorldBoundsBottom - RayPos, vec3(0,1,0)) / dot(RayDir, vec3(0,1,0) );
		float tp1 = (WorldBoundsBottom.y-RayPos.y)/RayDir.y;
		if ( tp1 > 0.0 )
		{
			MaxDistance = min( MaxDistance, tp1 );
			//res = vec2( tp1, 1.0 );
			FloorHit.x = tp1;	//	gr: why is floorhit distance wrong?
			MarchMissHit = vec4( RayPos + RayDir * FloorHit.x, FloorHit.y );
		}
	}
	
	for ( int s=0;	s<MaxSteps;	s++ )
	{
		vec3 Position = RayPos + RayDir * RayTime;
		
		//	intersect scene
		DistanceMaterial_t HitDistanceMaterial = map( Position);
		float HitDistance = HitDistanceMaterial.x;
	
	//MarchMissHit = vec4( RayPos + RayDir * HitDistanceMaterial.x, HitDistanceMaterial.y );;
	
		if ( HitDistance <= CloseEnough )
		{
			//float Heat = 1.0 - (float(s)/float(MaxSteps));
			return vec4( Position, HitDistanceMaterial.y );
		}
		RayTime += max( HitDistance, MinStep );
		RayTime -= StepBack;
		
		//	ray gone too far
		if (RayTime > MaxDistance)
			break;
	}
	
	return MarchMissHit;
}



vec3 calcNormal( in vec3 pos )
{
	vec2 e = vec2(1.0,-1.0)*0.5773;
	const float eps = 0.0005;
	return normalize( e.xyy * map( pos + e.xyy*eps ).x + 
					  e.yyx * map( pos + e.yyx*eps ).x + 
					  e.yxy * map( pos + e.yxy*eps ).x + 
					  e.xxx * map( pos + e.xxx*eps ).x );
}


void GetSceneIntersectionNew(vec3 RayPos,vec3 RayDir,out vec3 IntersectionPosition,out vec3 IntersectionNormal,out float IntersectionMaterial,out float IntersectionHeat)
{
	vec4 Int = GetSceneIntersection( RayPos, RayDir );
	IntersectionMaterial = Int.w;
	IntersectionHeat = 0.0;
	IntersectionPosition = Int.xyz;
	IntersectionNormal = calcNormal(IntersectionPosition);
}


vec4 GetLiquidColour(vec3 WorldPosition,vec3 WorldNormal)
{
	vec3 RumBright = vec3(255, 163, 64)/vec3(255,255,255);//	rum
	vec3 RumMidTone = vec3(181, 81, 4)/vec3(255,255,255);//	rum
	vec3 RumDark = vec3(184, 70, 0)/vec3(255,255,255);//	rum
	//ShadowColour = RumDark;
	vec3 Colour = RumMidTone;
		
	vec3 Normal = WorldNormal;
	//Colour = Range3( vec3(-1,-1,-1), vec3(1,1,1), Normal );
	vec3 DirToLight = normalize( LightPos-WorldPosition );
	float Dot = dot( DirToLight, Normal );
	//	tone mapping
	//Dot = ( abs(Dot) > 0.575 ) ? Dot : 0.0;
	//Dot *= Dot;	//	curve

	Colour = mix( Colour, RumBright, Dot );
	
	//Colour = NormalToRedGreen(TimeNormal);
	
	//	specular
	if ( Dot > 0.96 )
		Colour = vec3(1,1,1);
		
	return vec4( Colour, 1.0 );
}


vec4 GetBrownSugarColour(vec3 WorldPosition,vec3 WorldNormal)
{
	float Random = 0.0;
	/*
	//	sparkle
	vec3 LocalPosition = WorldPosition - IngredientPositions[0].xyz;
	
	//	divide into grid
	float CubeRadiusMin = 0.01;
	LocalPosition /= CubeRadiusMin * 2.0;
	float GridCount = 4.0; 
	LocalPosition = floor(LocalPosition * GridCount)/GridCount;
	
	float Random = rand(LocalPosition);
	//return vec4( Random, Random, Random, 1.0 );
*/
	vec3 DirToLight = normalize( LightPos-WorldPosition );
	float LightDot = dot( DirToLight, WorldNormal );
	
	LightDot -= Random * 0.5;

	if ( LightDot > 0.95 )
		return vec4( 1,1,1,1 );

	vec3 BrownSugarColourBright = min( vec3(1.0,1.0,1.0), BrownSugarColour * 1.8);

	vec3 Colour = BrownSugarColour;

	Colour = mix( Colour, BrownSugarColourBright, LightDot );

	return vec4(Colour,1.0);
}

//	todo: reflecton & refraction via material
//	w = alpha
//	todo: will later expand to allow reflection light
//			absorb shadows
//			ignore shadows
vec4 GetMaterialColour(float Material,vec3 WorldPosition,vec3 WorldNormal)
{
	if ( Material == Material_None )		return vec4(1,0,1,0);
	if ( Material == Material_Liquid )		return GetLiquidColour(WorldPosition,WorldNormal);
	if ( Material == Material_Floor )		return FloorColour;
	if ( Material == Material_BrownSugar )	return GetBrownSugarColour(WorldPosition,WorldNormal);
	if ( int(Material) == 4 )				return vec4(0,1,1,1);


	return vec4(0,0,1,1);
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

	vec3 IntersectionPosition,IntersectionNormal;
	float IntersectionMaterial;
	float IntersectionHeat;
	GetSceneIntersectionNew( RayPos, RayDir, IntersectionPosition, IntersectionNormal, IntersectionMaterial, IntersectionHeat );
	
	if ( DrawNormals )
	{
		vec3 Normal = IntersectionNormal;
		Normal = Range3( vec3(-1,-1,-1), vec3(1,1,1), Normal );
		gl_FragColor = vec4( Normal,1.0);
		return;
	}
	
	if ( DrawHeat )
	{
		float Shadow = HeatToShadow( IntersectionHeat );
		gl_FragColor = vec4( Shadow, Shadow, Shadow, 1.0);
		return;
	}
	
	vec4 Colour = vec4(0,1,0,1);
	float ShadowMult = 0.5;
	
	if ( IntersectionMaterial == Material_None )
		discard;
	
	//	hit liquid
	if ( IntersectionMaterial != Material_None )
	{
		Colour = GetMaterialColour( IntersectionMaterial, IntersectionPosition, IntersectionNormal );
	}
	
	
	/*
	else
	{
		//	check if we hit the floor
		vec4 FloorIntersection = GetSceneIntersection( RayPos, RayDir, true );
		if ( FloorIntersection.w <= 0.0 )
			discard;
			
		Alpha = 0.0;
		float GlassDistance = length( IntersectionPosition - RayPos );
		float FloorDistance = length( FloorIntersection.xyz - RayPos );
		if ( FloorDistance < GlassDistance )
		{
			IntersectionPosition = FloorIntersection.xyz;
			ShadowColour = vec3(0,0,0);
		}
	}
	*/
	
	//	do a hard shadow pass by shooting a ray to the sun
	//	todo: material property, can/not absorb shadow
	bool MaterialAbsorbShadow = true;
	if ( DrawShadows && MaterialAbsorbShadow )
	{
		vec3 DirToLight = normalize(LightPos - IntersectionPosition);
		//vec3 PositionToLight = Intersection.xyz+(Normal*0.002);
		vec3 PositionToLight = IntersectionPosition+(DirToLight*0.01);
		vec4 LightIntersection = GetSceneIntersection( PositionToLight, DirToLight );
		
		//	hit something
		if ( LightIntersection.w != Material_None )
		{
			//Colour = ShadowColour;
			Colour.xyz *= ShadowMult;
			Colour.w = 1.0;
		}
	}

	if ( Colour.w <= 0.0 )
		discard;
	gl_FragColor = Colour;
}
`;

export default Vert;
