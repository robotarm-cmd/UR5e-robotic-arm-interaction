function createUR5eMeshRenderer(canvas, meshData) {
  const gl=canvas.getContext('webgl',{alpha:true,antialias:true,premultipliedAlpha:false,preserveDrawingBuffer:true});
  if(!gl) throw new Error('此浏览器未启用 WebGL，请使用 Edge 或 Chrome。');
  function shader(type,source){const s=gl.createShader(type);gl.shaderSource(s,source);gl.compileShader(s);if(!gl.getShaderParameter(s,gl.COMPILE_STATUS))throw new Error(gl.getShaderInfoLog(s));return s;}
  const vertex=shader(gl.VERTEX_SHADER,`
    attribute vec3 position; attribute vec3 normal;
    uniform mat4 world; uniform vec3 target,right,up,toward;
    uniform vec2 clipScale;
    varying vec3 N;
    void main(){vec3 p=(world*vec4(position,1.0)).xyz-target;
      gl_Position=vec4(dot(p,right)*clipScale.x,dot(p,up)*clipScale.y,-dot(p,toward)/4.0,1.0);
      N=mat3(world)*normal;}
  `);
  const fragment=shader(gl.FRAGMENT_SHADER,`
    precision mediump float; varying vec3 N;
    uniform vec3 color,viewDirection;
    void main(){vec3 n=normalize(N);if(!gl_FrontFacing)n=-n;
      vec3 light=normalize(vec3(.4,-.6,1.0));
      float d=max(dot(n,light),0.0);
      float rim=max(dot(n,normalize(vec3(-.5,.4,.6))),0.0);
      vec3 halfVector=normalize(light+viewDirection);
      float spec=pow(max(dot(n,halfVector),0.0),45.0)*.28;
      gl_FragColor=vec4(clamp(color*(.43+.58*d+.22*rim)+vec3(spec),0.0,1.0),1.0);}
  `);
  const program=gl.createProgram();gl.attachShader(program,vertex);gl.attachShader(program,fragment);gl.linkProgram(program);
  if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw new Error(gl.getProgramInfoLog(program));
  const locations={};
  for(const name of ['world','target','right','up','toward','clipScale','color','viewDirection'])locations[name]=gl.getUniformLocation(program,name);
  const pos=gl.getAttribLocation(program,'position'),normalLoc=gl.getAttribLocation(program,'normal');
  function decode(s){const raw=atob(s),bytes=new Uint8Array(raw.length);for(let i=0;i<raw.length;i++)bytes[i]=raw.charCodeAt(i);return new Uint16Array(bytes.buffer);}
  function buffer(target,data){const b=gl.createBuffer();gl.bindBuffer(target,b);gl.bufferData(target,data,gl.STATIC_DRAW);return b;}
  const meshes=meshData.map(item=>{
    const packed=decode(item.vertices),indices=decode(item.indices),vertices=new Float32Array(packed.length),normals=new Float32Array(packed.length);
    for(let i=0;i<packed.length;i++){const k=i%3;vertices[i]=item.min[k]+packed[i]/65535*(item.max[k]-item.min[k]);}
    for(let f=0;f<indices.length;f+=3){
      const a=indices[f]*3,b=indices[f+1]*3,c=indices[f+2]*3;
      const ux=vertices[b]-vertices[a],uy=vertices[b+1]-vertices[a+1],uz=vertices[b+2]-vertices[a+2];
      const vx=vertices[c]-vertices[a],vy=vertices[c+1]-vertices[a+1],vz=vertices[c+2]-vertices[a+2];
      const n=[uy*vz-uz*vy,uz*vx-ux*vz,ux*vy-uy*vx];
      for(const start of [a,b,c])for(let k=0;k<3;k++)normals[start+k]+=n[k];
    }
    for(let i=0;i<normals.length;i+=3){const length=Math.hypot(normals[i],normals[i+1],normals[i+2])||1;for(let k=0;k<3;k++)normals[i+k]/=length;}
    return {frame:item.frame,color:item.material.slice(0,3),positions:buffer(gl.ARRAY_BUFFER,vertices),normals:buffer(gl.ARRAY_BUFFER,normals),indices:buffer(gl.ELEMENT_ARRAY_BUFFER,indices),count:indices.length};
  });
  function render(Ts,basis,camera,w,h){
    const dpr=Math.min(window.devicePixelRatio||1,2),pixelW=Math.round(w*dpr),pixelH=Math.round(h*dpr);
    if(canvas.width!==pixelW||canvas.height!==pixelH){canvas.width=pixelW;canvas.height=pixelH;}
    gl.viewport(0,0,canvas.width,canvas.height);gl.clearColor(0,0,0,0);gl.clearDepth(1);gl.enable(gl.DEPTH_TEST);gl.depthFunc(gl.LEQUAL);gl.disable(gl.BLEND);gl.disable(gl.CULL_FACE);gl.clear(gl.COLOR_BUFFER_BIT|gl.DEPTH_BUFFER_BIT);
    gl.useProgram(program);gl.enableVertexAttribArray(pos);gl.enableVertexAttribArray(normalLoc);
    for(const key of ['right','up','toward'])gl.uniform3fv(locations[key],basis[key]);
    gl.uniform3fv(locations.viewDirection,basis.toward);gl.uniform3fv(locations.target,camera.target);
    const scale=Math.min(w,h)/camera.span;gl.uniform2f(locations.clipScale,2*scale/w,2*scale/h);
    for(const mesh of meshes){
      const T=Ts[mesh.frame],matrix=new Float32Array(16);for(let row=0;row<4;row++)for(let col=0;col<4;col++)matrix[col*4+row]=T[row][col];
      gl.uniformMatrix4fv(locations.world,false,matrix);gl.uniform3fv(locations.color,mesh.color);
      gl.bindBuffer(gl.ARRAY_BUFFER,mesh.positions);gl.vertexAttribPointer(pos,3,gl.FLOAT,false,0,0);
      gl.bindBuffer(gl.ARRAY_BUFFER,mesh.normals);gl.vertexAttribPointer(normalLoc,3,gl.FLOAT,false,0,0);
      gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER,mesh.indices);gl.drawElements(gl.TRIANGLES,mesh.count,gl.UNSIGNED_SHORT,0);
    }
  }
  return {render};
}
