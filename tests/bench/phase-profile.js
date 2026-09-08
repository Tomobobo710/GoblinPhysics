/**
 * Where the time goes inside one step, on the same 500/3000 scene stack-and-scene.js measures.
 *
 *   node tests/bench/phase-profile.js              PGS
 *   node tests/bench/phase-profile.js --solver=pbd XPBD
 *
 * Wraps the phase entry points and accumulates wall time per phase over a timed window, after the
 * same 150-tick settle. Percentages are of measured step time, so they say which phase to attack -
 * the wrappers add their own small overhead, so treat the total as indicative, not a budget.
 */
var path = require('path');
var Goblin = require(path.join(__dirname, '..', '..', 'build', 'goblin.js'));
var SOLVER = (process.argv.slice(2).filter(function(a){return a.indexOf('--solver=')===0;})[0]||'').slice(9)||'pgs';
var SETTLE = 150, MEASURE = 200;

function mulberry32(a){return function(){a|=0;a=(a+0x6D2B79F5)|0;var t=Math.imul(a^(a>>>15),1|a);
 t=(t+Math.imul(t^(t>>>7),61|t))^t;return((t^(t>>>14))>>>0)/4294967296;};}

function buildScene(){
 var rand=mulberry32(1234);
 var solver=SOLVER==='pbd'?new Goblin.PBDSolver():new Goblin.IterativeSolver();
 var w=new Goblin.World(new Goblin.SAPBroadphase(),new Goblin.NarrowPhase(),solver);
 w.gravity=new Goblin.Vector3(0,-9.8,0);
 var perSide=55,half=100,tile=(half*2)/perSide;
 var comp=new Goblin.CompoundShape(),ident=new Goblin.Quaternion(0,0,0,1),zero=new Goblin.Vector3(0,0,0);
 for(var gz=0;gz<perSide;gz++)for(var gx=0;gx<perSide;gx++){
  var cx=-half+tile/2+gx*tile,cz=-half+tile/2+gz*tile,th=tile/2;
  var h0=Math.sin(cx*0.05)*0.6+Math.cos(cz*0.05)*0.6;
  comp.addChildShape(new Goblin.MeshShape([
   new Goblin.Vector3(cx-th,h0,cz-th),new Goblin.Vector3(cx+th,h0,cz-th),
   new Goblin.Vector3(cx+th,h0,cz+th),new Goblin.Vector3(cx-th,h0,cz+th)],[0,2,1,0,3,2]),zero,ident);}
 w.addRigidBody(new Goblin.RigidBody(comp,Infinity));
 for(var i=0;i<500;i++){var kind=i%3,b;
  var x=(rand()*2-1)*half*0.9,z=(rand()*2-1)*half*0.9,y=2+rand()*6;
  if(kind===0)b=new Goblin.RigidBody(new Goblin.BoxShape(0.4+rand()*0.3,0.4+rand()*0.3,0.4+rand()*0.3),5+rand()*10);
  else if(kind===1)b=new Goblin.RigidBody(new Goblin.CylinderShape(0.3+rand()*0.2,0.4+rand()*0.3),5+rand()*10);
  else b=new Goblin.RigidBody(new Goblin.ConeShape(0.3+rand()*0.2,0.5+rand()*0.3),5+rand()*10);
  b.position.set(x,y,z);
  b.rotation=new Goblin.Quaternion(rand()-0.5,rand()-0.5,rand()-0.5,1);
  w.addRigidBody(b);}
 return w;
}

var acc={},on=false;
function wrap(obj,name,label){
 var orig=obj[name];
 if(typeof orig!=='function')return;
 obj[name]=function(){
  if(!on)return orig.apply(this,arguments);
  var t0=process.hrtime.bigint();
  var r=orig.apply(this,arguments);
  acc[label]=(acc[label]||0)+Number(process.hrtime.bigint()-t0)/1e6;
  return r;};
}

wrap(Goblin.SAPBroadphase.prototype,'update','broadphase');
wrap(Goblin.NarrowPhase.prototype,'generateContacts','narrowphase');
wrap(Goblin.NarrowPhase.prototype,'midPhase','  └ midPhase (compound)');
wrap(Goblin.NarrowPhase.prototype,'updateContactManifolds','  └ manifold update');
wrap(Goblin.GjkEpa,'GJK','  └ GJK');
wrap(Goblin.GjkEpa,'EPA','  └ EPA');
if(SOLVER==='pbd'){
 wrap(Goblin.PBDSolver.prototype,'_solvePositions','solver: positions');
 wrap(Goblin.PBDSolver.prototype,'_solveVelocities','solver: velocities');
 wrap(Goblin.PBDSolver.prototype,'processContactManifolds','solver: processManifolds');
}else{
 wrap(Goblin.IterativeSolver.prototype,'processContactManifolds','solver: processManifolds');
 wrap(Goblin.IterativeSolver.prototype,'prepareConstraints','solver: prepare');
 wrap(Goblin.IterativeSolver.prototype,'resolveContacts','solver: resolveContacts');
 wrap(Goblin.IterativeSolver.prototype,'solveConstraints','solver: solveConstraints');
 wrap(Goblin.IterativeSolver.prototype,'applyConstraints','solver: applyConstraints');
}

var w=buildScene();
for(var s=0;s<SETTLE;s++)w.step(1/60);
on=true;
var t0=process.hrtime.bigint();
for(s=0;s<MEASURE;s++)w.step(1/60);
var total=Number(process.hrtime.bigint()-t0)/1e6;
on=false;

console.log('solver: '+SOLVER+'   '+MEASURE+' timed ticks after '+SETTLE+' settle');
console.log('total step time: '+(total/MEASURE).toFixed(3)+' ms/tick\n');
Object.keys(acc).sort(function(a,b){
 var ai=a.indexOf('└')===-1,bi=b.indexOf('└')===-1;
 if(ai!==bi)return 0;
 return acc[b]-acc[a];
}).forEach(function(k){
 var per=acc[k]/MEASURE;
 console.log('  '+k.padEnd(26)+per.toFixed(3)+' ms/tick   '+(100*acc[k]/total).toFixed(1)+'%');
});
