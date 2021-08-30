import * as PopMath from './PopEngineCommon/Math.js'
const Default = 'Drink.js';
export default Default;

import Pop from './PopEngineCommon/PopEngine.js'
import Params from './Params.js'
import {CreateCubeGeometry,CreateQuad3Geometry} from './PopEngineCommon/CommonGeometry.js'
import {ExtractShaderUniforms} from './PopEngineCommon/Shaders.js'

import * as DrinkShaderSource from './Drink/DrinkShader.glsl.js'
import * as BasicShaderSource from './Assets/BasicGeoShader.glsl.js'
import * as SdfMapShaderSource from './Drink/SdfMapShader.glsl.js'


let DrinkShader = null;
let DrinkGeo = null;
let DrinkGeoAttribs = null;
let BasicShader = null;
let BlitQuadGeo = null;
let SdfMapShader = null;
let BlitQuadGeoAttribs = null;

let DrinkBottom = null;
let DrinkTop = null;
let PendingPosition = null;
let DrinkRadius = 0.06;
let UserTime = 0;
let UserTop = null;

export function OnHoverMap(WorldPos,WorldRay,FirstDown)
{
}

export function OnUnclickMap()
{
	if ( PendingPosition )
	{
		if ( !DrinkBottom )
		{
			DrinkBottom = PendingPosition;
			PendingPosition = null;
		}	
		else if ( !DrinkTop )
		{
			DrinkTop = PendingPosition;
			PendingPosition = null;
		}
		else
		{
			UserTop = PendingPosition;
			PendingPosition = null;
		}
	}
}

export function OnClickMap(WorldPos,WorldRay,FirstDown)
{
	if ( !DrinkBottom )
	{
		//	need to get a hit for placing the bottom 
		if ( WorldPos )
		{
			//PendingPosition = WorldPos;
			//	immediatly place
			DrinkBottom = WorldPos;
			
			//	auto height
			DrinkTop = PopMath.Add3( DrinkBottom, [0,0.15,0] );
		}
	}
	//else if ( DrinkBottom )
	else if ( !DrinkTop && DrinkBottom )
	{
		//	need to find the Y above drinkbottom that we're pointing at
		//	so get two rays, drink up, and our ray
		//	then find the nearest point on the drink line
		const DrinkUp = [0,1,0];
		const Result = PopMath.GetRayRayIntersection3( DrinkBottom, DrinkUp, WorldRay.Start, WorldRay.Direction );
		let NewDrinkTop = PopMath.GetRayPositionAtTime( DrinkBottom, DrinkUp, Result.IntersectionTimeA );
		
		NewDrinkTop[1] = Math.max( DrinkBottom[1], NewDrinkTop[1] );
		DrinkTop = NewDrinkTop;
	}
	else
	{
		//	need to find the Y above drinkbottom that we're pointing at
		//	so get two rays, drink up, and our ray
		//	then find the nearest point on the drink line
		const DrinkUp = [0,1,0];
		const Result = PopMath.GetRayRayIntersection3( DrinkBottom, DrinkUp, WorldRay.Start, WorldRay.Direction );
		let NewDrinkTop = PopMath.GetRayPositionAtTime( DrinkBottom, DrinkUp, Result.IntersectionTimeA );
		
		NewDrinkTop[1] = Math.max( DrinkBottom[1], NewDrinkTop[1] );
		UserTop = NewDrinkTop;
		
		let UserBottomDistance = 0.1;
		let UserTopDistance = 0.3;
		let UserDistance = PopMath.Distance3( DrinkBottom, NewDrinkTop );
		UserTime = PopMath.RangeClamped( UserBottomDistance, UserTopDistance, UserDistance );
		//UserTime = PopMath.Clamp01( UserTime );
	}
}

export async function LoadAssets(RenderContext)
{
	if ( !DrinkGeo )
	{
		const Geometry = CreateCubeGeometry(0,1);
		DrinkGeo = await RenderContext.CreateGeometry(Geometry,undefined);
		DrinkGeoAttribs = Object.keys(Geometry);
	}
	
	if ( !BlitQuadGeo )
	{
		const Geometry = CreateQuad3Geometry(-1,1);
		BlitQuadGeo = await RenderContext.CreateGeometry(Geometry,undefined);
		BlitQuadGeoAttribs = Object.keys(Geometry);
	}
	
	if ( !DrinkShader && DrinkGeo )
	{
		const FragSource = DrinkShaderSource.Frag;
		const VertSource = DrinkShaderSource.Vert;
		const ShaderUniforms = ExtractShaderUniforms(VertSource,FragSource);
		DrinkShader = await RenderContext.CreateShader(VertSource,FragSource,ShaderUniforms,DrinkGeoAttribs);
	}
	
	if ( !BasicShader && DrinkGeo )
	{
		const FragSource = BasicShaderSource.Frag;
		const VertSource = BasicShaderSource.Vert;
		const ShaderUniforms = ExtractShaderUniforms(VertSource,FragSource);
		BasicShader = await RenderContext.CreateShader(VertSource,FragSource,ShaderUniforms,DrinkGeoAttribs);
	}
	
	if ( !SdfMapShader && BlitQuadGeo )
	{
		const FragSource = SdfMapShaderSource.Frag;
		const VertSource = SdfMapShaderSource.Vert;
		const ShaderUniforms = ExtractShaderUniforms(VertSource,FragSource);
		SdfMapShader = await RenderContext.CreateShader(VertSource,FragSource,ShaderUniforms,BlitQuadGeoAttribs);
	}
}


class PhysicsSet_t
{
	constructor(Bottom,Top,RadiusZero,RadiusOne,Count)
	{
		let OneBottom = Top.slice();
		let OneTop = PopMath.Add3( Top, [0,0.3,0] );
		let ZeroTop = PopMath.Subtract3( Top, [0,RadiusZero,0] );
		
		this.ZeroPositions = RandomPositionsInsideCylinder(Bottom,ZeroTop,RadiusZero,Count);
		this.OnePositions = RandomPositionsInsideCylinder(OneBottom,OneTop,RadiusOne,Count);
		
		this.Positions = this.ZeroPositions.slice();
		this.Velocities = new Array(this.Positions.length).fill(0);
		
		this.Positions = new Float32Array(this.Positions);
		this.Velocities = new Float32Array(this.Velocities);
	}
	
	Update(Params)
	{
		//	update each pos
		for ( let fi=0;	fi<this.Positions.length;	fi+=4 )
		{
			let TimeDelta = Params.Speed / 60.0;
			let Friction = Params.Friction;
			let Spring = Params.Spring;
			let BrownianForce = Params.BrownianForce;
			 
			//let i = fi / 4;
			let v_xyz = this.Velocities.slice(fi,fi+3);
			let xyz = this.Positions.slice(fi,fi+3);
			
			//let SpringTarget = Params.SpringTarget;
			let SpringTargetA = this.ZeroPositions.slice(fi,fi+3);
			let SpringTargetB = this.OnePositions.slice(fi,fi+3);
			let SpringTarget = PopMath.Lerp3( SpringTargetA, SpringTargetB, UserTime );
		
			//	spring back to 0
			v_xyz[0] += (SpringTarget[0]-xyz[0]) * Spring;
			v_xyz[1] += (SpringTarget[1]-xyz[1]) * Spring;
			v_xyz[2] += (SpringTarget[2]-xyz[2]) * Spring;
			
			v_xyz[0] += (Math.random()-0.5) * BrownianForce;
			v_xyz[1] += (Math.random()-0.5) * BrownianForce;
			v_xyz[2] += (Math.random()-0.5) * BrownianForce;
			v_xyz[0] *= 1.0 - Friction;
			v_xyz[1] *= 1.0 - Friction;
			v_xyz[2] *= 1.0 - Friction;
			
			
			xyz[0] += v_xyz[0] * TimeDelta;
			xyz[1] += v_xyz[1] * TimeDelta;
			xyz[2] += v_xyz[2] * TimeDelta;
			
			this.Velocities.set( v_xyz, fi );
			this.Positions.set( xyz, fi );
		}
	}
}

let LiquidPhsyics = null;
let LiquidSphereCount = 25;

let IngredientPhsyics = null;
let IngredientCount = 3;

function RandomPositionsInsideCylinder(Bottom,Top,Radius,Count)
{
	let Positions = [];
	
	//	might be good later to do random xyz in volume, then clip with sdf which we may prefer in future
	
	for ( let i=0;	i<Count;	i++ )
	{
		let r = Math.random();
		let y = Math.random();
		let a = PopMath.DegToRad( Math.random() * 360.0 );

		//	if radius is truly uniform, then we'll be more dense in the middle, so weight random radius towards edge
		r = 1.0 - ( r*r );
		r *= Radius;
		
		let x = Math.cos(a) * r;
		let z = Math.sin(a) * r;
		
		//	todo: xz should be cross bottom->top in case it's not straight up!
		let xyz = PopMath.Lerp3( Bottom, Top, y );
		xyz[0] += x;
		xyz[2] += z;
		let w = Math.random();

		Positions.push( ...xyz, w );
	}
	
	return Positions;
}


function CreatePhysicsSet(VerletCount,RadiusZero,RadiusOne)
{
	if ( !DrinkTop && !DrinkBottom )
		return null;

	let Set = new PhysicsSet_t( DrinkBottom, DrinkTop, RadiusZero, RadiusOne, VerletCount );
	return Set;
}



export function Update()
{
	if ( !LiquidPhsyics )
		LiquidPhsyics = CreatePhysicsSet( LiquidSphereCount, DrinkRadius, DrinkRadius*2.5 );
	
	if ( !IngredientPhsyics )
		IngredientPhsyics = CreatePhysicsSet( IngredientCount, 0, DrinkRadius );

	
	const LiquidParams = {};
	LiquidParams.Speed = 1.6;
	LiquidParams.Friction = 0.20;
	LiquidParams.Spring = 0.90;//0.2;
	LiquidParams.BrownianForce = 0.01;//0.15;
	LiquidParams.SpringTarget = UserTop ? UserTop.slice() : [0,0.0,0];
	if ( LiquidPhsyics )
		LiquidPhsyics.Update( LiquidParams );
	
		
	const IngredientParams = {};
	IngredientParams.Speed = 1.0;
	IngredientParams.Friction = 0.10;
	IngredientParams.Spring = 0.9;
	IngredientParams.BrownianForce = 0;
	IngredientParams.SpringTarget = PopMath.Add3( [0,0.0,0], UserTop ? UserTop.slice() : [0,0.0,0] );
	if ( IngredientPhsyics )
		IngredientPhsyics.Update( IngredientParams );
	

}

function GetNormalisedTime()
{
	const LoopDurationMs = Params.LoopDurationSecs * 1000; 
	let TimeMs = Pop.GetTimeNowMs()
	TimeMs %= LoopDurationMs;
	
	//	normalise
	let TimeNorm  = TimeMs / LoopDurationMs;
	
	//	ping pong
	if ( TimeNorm >= 0.5 )
		TimeNorm = PopMath.Range( 1.0, 0.5, TimeNorm );
	else
		TimeNorm = PopMath.Range( 0.0, 0.5, TimeNorm );

	return TimeNorm;
}

let SdfSlicesTextures = null;
let SdfSliceCount = 10;
const DrinkClipRadius = 0.5;

export function GetRenderCommands(CameraUniforms,Camera,Assets)
{
	let Commands = [];
	
	const RealTime = GetNormalisedTime();

	let Bottom = DrinkBottom;
	let Top = PendingPosition || DrinkTop || DrinkBottom;
	
/*
	if ( !SdfSlicesTextures && DrinkBottom && DrinkTop )
	{
		SdfSlicesTextures = [];
		for ( let s=0;	s<SdfSliceCount;	s++ )
		{
			let SdfTexture = new Pop.Image();
			const Pixels = new Float32Array( 1024*1024*4 );
			SdfTexture.WritePixels( 1024, 1024, Pixels, 'Float4' );
			SdfSlicesTextures.push(SdfTexture);
		}
	}
	
	*/
	//	make array of slice quad positions
	let SliceQuadMinMaxs = [];	//	[0,0,0,	1,1,1
	if ( Top && Bottom )
	{
		for ( let s=0;	s<SdfSliceCount;	s++ )
		{
			let ClipZ = DrinkClipRadius * 0.3;
			//let MaxZ = DrinkRadius + ClipZ;
			let MaxZ = DrinkRadius + 0.1;
			let MinZ = -MaxZ;
			let XOff = DrinkRadius + DrinkClipRadius*0.5;
			let YUp = DrinkClipRadius;
			let YDown = 0;
			//function AddSlice(CenterX,CenterY,CenterZ
			let z = PopMath.Lerp( MinZ, MaxZ, s/SdfSliceCount );
			let Min = PopMath.Add3( Top, [-XOff,YUp,z] );
			let Max = PopMath.Add3( Bottom, [XOff,YDown,z] );
			SliceQuadMinMaxs.push( [Min, Max] );
		}
	}
	
	for ( let s=0;	s<SliceQuadMinMaxs.length;	s++ )
	{
		const SliceQuadMinMax = SliceQuadMinMaxs[s];
		const Uniforms = Object.assign({},CameraUniforms);
		Uniforms.WorldQuadMin = SliceQuadMinMax[0];
		Uniforms.WorldQuadMax = SliceQuadMinMax[1];
		Uniforms.BlitProjection = 0;

		//	sdf data
		Uniforms.DrinkRadius = DrinkRadius;
		Uniforms.TimeNormal = UserTime;
		Uniforms.RealTime = RealTime;
	
			
		if ( LiquidPhsyics )
			Uniforms.LiquidSpherePositions = LiquidPhsyics.Positions;
		if ( IngredientPhsyics )
			Uniforms.IngredientPositions = IngredientPhsyics.Positions;
			
		Commands.push( ['Draw',BlitQuadGeo,SdfMapShader,Uniforms] );

	}
	
	/*
	//	render slices
	if ( SdfTexture )
	{
		Commands.push( ['SetRenderTarget',SdfTexture,[0,0,1,1]] );
		
		const Uniforms = Object.assign({},CameraUniforms);
		Uniforms.WorldBoundsBottom = PopMath.Add3( Top, [0,0.1,0] );
		Uniforms.WorldBoundsTop = PopMath.Add3( Top, [0,0.4,0] );
		Uniforms.BoundsRadius = 0.1;
		Uniforms.DrinkRadius = DrinkRadius;
		Uniforms.TimeNormal = UserTime;
		Uniforms.RealTime = RealTime;
			
		if ( LiquidPhsyics )
			Uniforms.LiquidSpherePositions = LiquidPhsyics.Positions;
		if ( IngredientPhsyics )
			Uniforms.IngredientPositions = IngredientPhsyics.Positions;
			
		Commands.push( ['Draw',BlitQuadGeo,SdfMapShader,Uniforms] );

		//	restore render target
		Commands.push( ['SetRenderTarget',null] );
		//return Commands;
	}
	*/
	
	
	function DrawCube(Position,Size=0.01)
	{
		if ( !Position )
			return;
		const Uniforms = Object.assign({},CameraUniforms);
		//const Position = TrackPoint.Position.slice();
		Uniforms.LocalToWorldTransform = PopMath.CreateTranslationScaleMatrix(Position,[Size,Size,Size]);
		Uniforms.Selected = false;//TrackPoint.Selected;
		Uniforms.Texture = SdfTexture;
		Commands.push( ['Draw',DrinkGeo,BasicShader,Uniforms] );
	}
	
	//if ( SdfTexture )
		//DrawCube(Bottom,0.1);
	
	//if ( false )
	{
		//DrawCube(Bottom);
		//DrawCube(Top);
		//DrawCube(UserTop);
		
		if ( Bottom && Top )
		{
			const Uniforms = Object.assign({},CameraUniforms);
			Uniforms.LocalToWorldTransform = PopMath.CreateIdentityMatrix();
			Uniforms.WorldBoundsBottom = Bottom;
			Uniforms.WorldBoundsTop = Top;
			Uniforms.BoundsRadius = DrinkClipRadius;
			Uniforms.DrinkRadius = DrinkRadius;
			Uniforms.TimeNormal = UserTime;
			
			if ( LiquidPhsyics )
				Uniforms.LiquidSpherePositions = LiquidPhsyics.Positions;
			if ( IngredientPhsyics )
				Uniforms.IngredientPositions = IngredientPhsyics.Positions;
			
			Commands.push( ['Draw',DrinkGeo,DrinkShader,Uniforms] );
			//Commands.push( ['Draw',BlitQuadGeo,BasicShader,Uniforms] );
		}
	}

	return Commands;
}
