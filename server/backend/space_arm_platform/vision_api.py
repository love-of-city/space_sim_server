"""Read-only vision diagnostics on the authenticated platform API."""
from fastapi import HTTPException
from fastapi.responses import Response

def install_vision_routes(app, *, vision, wrist_vision, hub, scenes):
    def current_stereo():
        state = vision.status()
        observation = hub.latest_observation
        if (hub.resetting or not hub.connected or scenes.status().get('phase') != 'running'
                or not observation or not observation.render_session_id
                or state['session_id'] != observation.render_session_id):
            vision.invalidate()
        return vision.status()

    @app.get('/api/vision/status')
    async def vision_status():
        return current_stereo()


    @app.get('/api/vision/wrist/status')
    async def wrist_vision_status():
        if hub.resetting or not hub.connected or scenes.status().get('phase')!='running':
            wrist_vision.invalidate('waiting','场景未就绪或正在复位')
        observation=hub.latest_observation
        state=wrist_vision.status()
        if (observation and observation.render_session_id and state['session_id']
                and state['session_id']!=observation.render_session_id):
            wrist_vision.invalidate('waiting','等待新场景图像')
            state=wrist_vision.status()
        return state


    @app.get('/api/vision/diagnostics')
    async def vision_diagnostics():
        current_stereo()
        return vision.diagnostics()


    @app.get('/api/vision/image/{eye}')
    async def vision_image(eye: str):
        if eye not in {'left', 'right'}:
            raise HTTPException(status_code=404, detail='unknown stereo eye')
        state = current_stereo()
        image = None if state['stale'] else vision.latest_image(eye)
        if image is None:
            raise HTTPException(status_code=404, detail='stereo camera has not produced an image')
        return Response(image, media_type='image/png', headers={'Cache-Control': 'no-store'})
