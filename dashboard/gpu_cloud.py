"""GTK OpenGL 3.3 point/surface renderer; uploads only when a frame changes."""
from pathlib import Path
import sys,time,ctypes
sys.path.insert(0,str(Path(__file__).with_name('.deps')))
import numpy as np
from gi.repository import Gtk
from OpenGL import GL as gl
from OpenGL.GL.shaders import compileProgram,compileShader

VERTEX='''#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 color;
uniform mat3 rotation;
uniform vec3 target;
uniform float distanceToTarget, focal, aspect, pointSize;
out vec3 tint;
void main(){
 vec3 p=rotation*(position-target);
 float depth=distanceToTarget-p.z;
 float n=0.03, f=60.0;
 gl_Position=vec4(focal*p.x/aspect,focal*p.y,((f+n)/(f-n))*depth-2.0*f*n/(f-n),depth);
 gl_PointSize=pointSize;
 tint=color;
}'''
FRAGMENT='''#version 330 core
in vec3 tint;
uniform bool points;
out vec4 pixel;
void main(){
 if(points && length(gl_PointCoord-vec2(0.5))>0.5)discard;
 pixel=vec4(tint,1.0);
}'''

class GPUCloud(Gtk.GLArea):
    def __init__(self,viewer):
        super().__init__()
        self.viewer=viewer;self.ready=False;self.failed=False;self.ms=0;self.upload_key=None
        self.set_required_version(3,3);self.set_has_depth_buffer(True);self.set_auto_render(False)
        self.connect('realize',self.on_realize);self.connect('render',self.on_render)
        self.connect('unrealize',self.on_unrealize)
    def fail(self,error):
        from gi.repository import GLib
        self.failed=True
        GLib.idle_add(self.viewer.gpu_failed,str(error))
    def on_realize(self,*_):
        self.make_current()
        if self.get_error():self.fail(self.get_error());return
        try:
            self.program=compileProgram(compileShader(VERTEX,gl.GL_VERTEX_SHADER),compileShader(FRAGMENT,gl.GL_FRAGMENT_SHADER))
            self.vao=gl.glGenVertexArrays(1);self.vbo=gl.glGenBuffers(1);self.ebo=gl.glGenBuffers(1)
            gl.glBindVertexArray(self.vao);gl.glBindBuffer(gl.GL_ARRAY_BUFFER,self.vbo)
            for i in range(2):
                gl.glEnableVertexAttribArray(i);gl.glVertexAttribPointer(i,3,gl.GL_FLOAT,False,24,ctypes.c_void_p(i*12))
            self.uniforms={k:gl.glGetUniformLocation(self.program,k) for k in
                ('rotation','target','distanceToTarget','focal','aspect','pointSize','points')}
            self.renderer_name=gl.glGetString(gl.GL_RENDERER).decode()
            self.ready=True
            print('OpenGL renderer:',self.renderer_name,flush=True)
        except Exception as exc:self.fail(exc)
    def on_render(self,*_):
        if not self.ready or self.failed:return True
        try:
            start=time.monotonic();v=self.viewer
            w,h=self.get_allocated_width()*self.get_scale_factor(),self.get_allocated_height()*self.get_scale_factor()
            gl.glViewport(0,0,w,h);gl.glClearColor(.035,.05,.075,1)
            gl.glEnable(gl.GL_DEPTH_TEST);gl.glEnable(gl.GL_PROGRAM_POINT_SIZE)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT|gl.GL_DEPTH_BUFFER_BIT)
            if v.frame is None:return True
            gl.glUseProgram(self.program);gl.glBindVertexArray(self.vao)
            key=(v.seq,v.confidence_min,v.surface_jump,v.range_lo,v.range_hi)
            if key!=self.upload_key:
                from cloud_geometry import surface_triangles
                xyz=v.frame['xyz'];rgb=v.all_rgb
                # Grid and sensor axes are guides, not measured points.
                floor=-.6 if v.floor_y is None else min(v.floor_y,-.05)
                cx,cz=np.round(v.cam.target[[0,2]])
                guides=[]
                for k in np.arange(-3,3.01,.5):
                    guides.extend([[cx+k,floor,cz-3,.16,.20,.25],[cx+k,floor,cz+3,.16,.20,.25],
                                   [cx-3,floor,cz+k,.16,.20,.25],[cx+3,floor,cz+k,.16,.20,.25]])
                for axis,color in (([.25,0,0],[.9,.3,.3]),([0,.25,0],[.3,.9,.4]),([0,0,-.25],[.3,.6,1])):
                    guides.extend([[0,0,0,*color],[*axis,*color]])
                self.guide_start=len(xyz);self.guide_count=len(guides)
                data=np.ascontiguousarray(np.vstack((np.column_stack((xyz,rgb)),guides)),dtype=np.float32)
                gl.glBindBuffer(gl.GL_ARRAY_BUFFER,self.vbo);gl.glBufferData(gl.GL_ARRAY_BUFFER,data.nbytes,data,gl.GL_DYNAMIC_DRAW)
                self.point_ids=np.flatnonzero(v.valid).astype(np.uint32)
                self.tri_ids=surface_triangles(xyz,v.valid,v.surface_jump).ravel()
                indices=np.concatenate((self.point_ids,self.tri_ids))
                gl.glBindBuffer(gl.GL_ELEMENT_ARRAY_BUFFER,self.ebo);gl.glBufferData(gl.GL_ELEMENT_ARRAY_BUFFER,indices.nbytes,indices,gl.GL_DYNAMIC_DRAW)
                self.upload_key=key
            u=self.uniforms
            gl.glUniformMatrix3fv(u['rotation'],1,True,v.cam.rotation())
            gl.glUniform3fv(u['target'],1,v.cam.target)
            gl.glUniform1f(u['distanceToTarget'],v.cam.dist);gl.glUniform1f(u['focal'],1/np.tan(v.cam.FOV/2))
            gl.glUniform1f(u['aspect'],w/max(h,1));gl.glUniform1f(u['pointSize'],v.point_size*self.get_scale_factor())
            if v.surface_mode and len(self.tri_ids):
                gl.glUniform1i(u['points'],False)
                gl.glDrawElements(gl.GL_TRIANGLES,len(self.tri_ids),gl.GL_UNSIGNED_INT,ctypes.c_void_p(self.point_ids.nbytes))
            else:
                gl.glUniform1i(u['points'],True)
                gl.glDrawElements(gl.GL_POINTS,len(self.point_ids),gl.GL_UNSIGNED_INT,ctypes.c_void_p(0))
            gl.glUniform1i(u['points'],False)
            gl.glDrawArrays(gl.GL_LINES,self.guide_start,self.guide_count)
            self.ms=(time.monotonic()-start)*1000
            v.draw_times.append(self.ms)
        except Exception as exc:self.fail(exc)
        return True
    def on_unrealize(self,*_):
        if not self.ready:return
        self.make_current()
        if not self.get_error():
            gl.glDeleteBuffers(2,[self.vbo,self.ebo]);gl.glDeleteVertexArrays(1,[self.vao]);gl.glDeleteProgram(self.program)
        self.ready=False
