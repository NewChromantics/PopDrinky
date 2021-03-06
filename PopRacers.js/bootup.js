import Pop from './PopEngineCommon/PopEngine.js'

//	allow use as module on web...
const Default = 'Bootup.js Module';
export default Default;

import * as RenderScene from './RenderScene.js'
//import * as Xr from './XrFrame.js'

//	gr: for now prevent creation of window on web
const MainWindow = Pop.GetPlatform()=='Web' ? null : new Pop.Gui.Window(null);

import * as Gui from './Gui.js'


function SaveMeshes()
{
	let Geos = RenderScene.GetWorldGeos();
	Geos = Geos.map( wg => wg.Anchor.Geometry );

	
	
}
Gui.SetEvent('SaveMeshes',SaveMeshes);


async function CreateMainWindowRenderContext(RenderViewName)
{
	for ( let i=0;	i<99999;	i++ )
	{
		try
		{
			const ViewWindow = MainWindow;
			if ( RenderViewName != 'RenderView' )
				throw 'x';
			const RenderView = new Pop.Gui.RenderView(ViewWindow,RenderViewName);
			const Sokol = new Pop.Sokol.Context(RenderView);
			Sokol.RenderView = RenderView;
			Sokol.RenderView.OnMouseDown = RenderScene.OnMouseDown.bind(RenderView);
			Sokol.RenderView.OnMouseMove = RenderScene.OnMouseMove.bind(RenderView);
			Sokol.RenderView.OnMouseScroll = RenderScene.OnMouseScroll.bind(RenderView);
			//	until renderview gets a rect func
			Sokol.RenderView.GetScreenRect = Sokol.GetScreenRect.bind(Sokol);
			return Sokol;
		}
		catch(e)
		{
			Pop.Debug(`Failed to make render context (${RenderViewName}); ${e}...`);
			await Pop.Yield(5000);
		}
	}
	throw `Couldn't make render context`;
}



async function WindowRenderThread(RenderViewName,DoRender)
{
	//	new sokol renderer
	const RenderThrottleMs = 1;
	const Sokol = await CreateMainWindowRenderContext(RenderViewName);
	const RenderView = Sokol.RenderView;
	
	Pop.Debug(`Created context ${RenderViewName}`);

	let FrameCount = 0;

	while (Sokol)
	{
		try
		{
			RenderScene.Update();
			
			const ScreenRect = RenderView.GetScreenRect();
			await RenderScene.LoadAssets(Sokol);
			const Commands = RenderScene.GetRenderCommands(FrameCount,ScreenRect);
			await Sokol.Render(Commands);
			RenderScene.PostRender();
			FrameCount++;
			await Pop.Yield(RenderThrottleMs);
		}
		catch(e)
		{
			Pop.Debug(`Renderloop error; ${e}`);
			await Pop.Yield(1000);
		}
	}
}
WindowRenderThread('RenderView').catch(Pop.Warning);
//	testing 2nd screen on ios
//WindowRenderThread('ExternalScreen').catch(Pop.Warning);


if ( Pop.GetPlatform()=='Web' )
{
	RenderScene.CreateTestPlane();
}

