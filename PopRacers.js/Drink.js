import * as PopMath from './PopEngineCommon/Math.js'
const Default = 'Drink.js';
export default Default;

import Pop from './PopEngineCommon/PopEngine.js'
import Params from './Params.js'
import {CreateCubeGeometry} from './PopEngineCommon/CommonGeometry.js'
import * as DrinkShaderSource from './Drink/DrinkShader.glsl.js'
import {ExtractShaderUniforms} from './PopEngineCommon/Shaders.js'


let DrinkShader = null;
let DrinkGeo = null;
let DrinkGeoAttribs = null;

let DrinkBottom = null;
let DrinkTop = null;
let PendingPosition = null;

export function OnHoverMap(WorldPos,WorldRay,FirstDown)
{
}

export function OnUnclickMap()
{
	if ( PendingPositon )
	{
		if ( !DrinkBottom )
		{
			DrinkBottom = PendingPosition;
			PendingPosition = null;
		}	
		else
		{
			DrinkTop = PendingPosition;
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
	else if ( DrinkBottom )
	//else if ( !DrinkTop && DrinkBottom )
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
}

export async function LoadAssets(RenderContext)
{
	if ( !DrinkGeo )
	{
		const Geometry = CreateCubeGeometry(0,1);
		DrinkGeo = await RenderContext.CreateGeometry(Geometry,undefined);
		DrinkGeoAttribs = Object.keys(Geometry);
	}
	
	if ( !DrinkShader && DrinkGeo )
	{
		const FragSource = DrinkShaderSource.Frag;
		const VertSource = DrinkShaderSource.Vert;
		const ShaderUniforms = ExtractShaderUniforms(VertSource,FragSource);
		DrinkShader = await RenderContext.CreateShader(VertSource,FragSource,ShaderUniforms,DrinkGeoAttribs);
	}
}


let LiquidSpherePositons = null;
let LiquidSphereVelocties = null;
let LiquidSphereDim = 4;

export function Update()
{
	if ( !LiquidSpherePositons )
	{
		LiquidSpherePositons = [];
		LiquidSphereVelocties = [];
		for ( let x=0;	x<LiquidSphereDim;	x++ )
		{
			for ( let y=0;	y<LiquidSphereDim;	y++ )
			{
				for ( let z=0;	z<LiquidSphereDim;	z++ )
				{
					let xf = x / (LiquidSphereDim-1);
					let yf = y / (LiquidSphereDim-1);
					let zf = z / (LiquidSphereDim-1);
					let PositionScale = 0.1;
					xf *= PositionScale;
					yf *= PositionScale;
					zf *= PositionScale;
					let Radius = Math.random();
					LiquidSpherePositons.push( xf, yf, zf, Radius );
					LiquidSphereVelocties.push(0,0,0,0);
				}
			}
		}
		LiquidSpherePositons = new Float32Array(LiquidSpherePositons);
		LiquidSphereVelocties = new Float32Array(LiquidSphereVelocties);
	}
	
	//	update each pos
	for ( let fi=0;	fi<LiquidSpherePositons.length;	fi+=4 )
	{
		let TimeDelta = 1.0 / 60.0;
		let Friction = 0.03;
		let Spring = 0.2;
		let BrownianForce = 0.20;
		 
		//let i = fi / 4;
		let v_xyz = LiquidSphereVelocties.slice(fi,fi+3);
		let xyz = LiquidSpherePositons.slice(fi,fi+3);
		
		let SpringTarget = DrinkTop ? DrinkTop.slice() : [0,0.0,0];
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
		
		LiquidSphereVelocties.set( v_xyz, fi );
		LiquidSpherePositons.set( xyz, fi );
	}
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

export function GetRenderCommands(CameraUniforms,Camera,Assets)
{
	let Commands = [];
	
	const Time = GetNormalisedTime();
	
	function DrawCube(Position)
	{
		if ( !Position )
			return;
		const Uniforms = Object.assign({},CameraUniforms);
		//const Position = TrackPoint.Position.slice();
		Uniforms.LocalToWorldTransform = PopMath.CreateTranslationMatrix(...Position);
		Uniforms.Selected = false;//TrackPoint.Selected;
		Commands.push( ['Draw',DrinkGeo,DrinkShader,Uniforms] );
	}
	
	{
		let Bottom = DrinkBottom;
		let Top = PendingPosition || DrinkTop || DrinkBottom;
		//DrawCube(Bottom);
		//DrawCube(Top);
		
		if ( Bottom && Top )
		{
			const Uniforms = Object.assign({},CameraUniforms);
			Uniforms.LocalToWorldTransform = PopMath.CreateIdentityMatrix();
			Uniforms.WorldBoundsBottom = Bottom;
			Uniforms.WorldBoundsTop = Top;
			Uniforms.BoundsRadius = 0.8;
			Uniforms.DrinkRadius = 0.06;
			Uniforms.TimeNormal = Time * Time;
			Uniforms.LiquidSpherePositons = LiquidSpherePositons;
			Commands.push( ['Draw',DrinkGeo,DrinkShader,Uniforms] );
		}
	}

	return Commands;
}
