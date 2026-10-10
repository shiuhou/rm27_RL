/** Offline JSONL evidence audit; does not launch firmware or touch hardware. */
import * as fs from 'node:fs/promises';
import {createHash} from 'node:crypto';
const runs = [
 ['phase4-sitl-hover-01',null],['phase4-sitl-hover-02',null],
 ['phase4-sitl-hover-attitude-01',null],
 ['phase4-sitl-maneuver-truth-01','phase4-eval-rate-truth-01'],
 ['phase4-sitl-maneuver-telemetry-01',null],
 ['phase4-sitl-attitude-truth-01','phase4-eval-attitude-truth-01'],
 ['phase4-sitl-attitude-telemetry-01','phase4-eval-attitude-telemetry-01'],
 ['phase4-unshaped-truth-01',null],['phase4-unshaped-telemetry-01',null]];
const squareError=(a,b)=>a.reduce((s,x,i)=>s+(x-b[i])**2,0);
const norm=a=>Math.hypot(...a);
const mean=a=>a.reduce((s,x)=>s+x,0)/a.length;
const quantile=(a,q)=>{a=[...a].sort((a,b)=>a-b);return a[Math.floor((a.length-1)*q)];};
export function interpolate(frames,t,key){
 if(t<frames[0].sim_time_s||t>frames.at(-1).sim_time_s)return null;
 let lo=0,hi=frames.length-1;
 while(hi-lo>1){let m=(lo+hi)>>1;if(frames[m].sim_time_s<=t)lo=m;else hi=m;}
 const a=frames[lo],b=frames[hi],u=(t-a.sim_time_s)/(b.sim_time_s-a.sim_time_s);
 return a.truth[key].map((x,i)=>x+(b.truth[key][i]-x)*u);
}
export async function audit(root,output){
 await fs.mkdir(output);const hashes={},results=[];
 async function read(file){const b=await fs.readFile(root+'/'+file);hashes[file]=createHash('sha256').update(b).digest('hex');return b.toString();}
 async function json(file){return JSON.parse(await read(file));}
 for(const [name,external] of runs){
  const dir='artifacts/'+name,s=await json(dir+'/summary.json');
  const launch=await json(dir+'/launch.json');
  let c=s.contact_evaluation??(external?await json('artifacts/'+external+'/summary.json'):null);
  const r={name,mode:launch.mode,observation:launch.observation,action_layer:launch.action_layer,
   original_passed:s.passed,completed:!!s.completed,task_passed:c?c.passed:null,
   binary_sha256:launch.binary_sha256,runner_failure:s.failure??null,
   contact:c?{time_s:c.first_contact?.time_s,error_m:c.first_contact?.position_error_m,margin_m:c.first_contact?.region_margin_m,
    attitude_error_deg:c.first_contact?.attitude_error_deg,normal_speed_m_s:c.first_contact?.normal_speed_m_s,
    rate_rad_s:c.first_contact?.body_rate_rad_s,valid:c.valid_contact,separated:c.separated,secondary:c.secondary_contact,
    recovery_time_s:c.recovery_time_s,confirmed_recovery_s:c.time_to_confirmed_recovery_s,
    max_post_rate_rad_s:c.max_post_body_rate_rad_s,altitude_loss_m:c.altitude_loss_m,saturation_fraction:c.saturation_fraction,
    failures:c.failure_modes,replay_error:c.replay_max_state_error}:null};
  if(s.free_flight_s){
   const frames=(await read(dir+'/frames.jsonl')).trim().split('\n').map(JSON.parse);
   // MAVLink optional NaNs were serialized by Python; normalize only bare numeric tokens.
   let nonfinite=0;
   const messages=(await read(dir+'/mavlink.jsonl')).trim().split('\n').map(line=>JSON.parse(line.replace(/([:\[,]\s*)(?:NaN|-?Infinity)(?=\s*[,}\]])/g,(_,prefix)=>{nonfinite++;return prefix+'null';})));
   r.optional_nonfinite_tokens=nonfinite;
   const selected=frames.filter((f,i)=>i===0||f.estimate.timestamp!==frames[i-1].estimate.timestamp);
   const errors=[];
   for(const f of selected){const p=interpolate(frames,f.estimate.timestamp,'position_nwu'),v=interpolate(frames,f.estimate.timestamp,'velocity_nwu');if(!p)continue;
    errors.push([squareError(f.estimate.position,f.truth.position_nwu),squareError(f.estimate.position,p),squareError(f.estimate.velocity,f.truth.velocity_nwu),squareError(f.estimate.velocity,v)]);}
   const channels={};
   for(const kind of ['LOCAL_POSITION_NED','ATTITUDE_QUATERNION']){
    const rows=messages.filter(x=>x.received_sim_s>=s.release_time_s&&x.message.mavpackettype===kind),dt=[];
    for(let i=1;i<rows.length;i++)dt.push((rows[i].message.time_boot_ms-rows[i-1].message.time_boot_ms)*.001);
    channels[kind]={packets:rows.length,source_interval_s:{min:Math.min(...dt),median:quantile(dt,.5),max:Math.max(...dt)}};
   }
   const after=frames.filter(f=>f.sim_time_s-s.release_time_s>2);
   const rateRms=Math.sqrt(mean(after.map(f=>norm(f.truth.body_rate_flu)**2)));
   if(launch.mode==='hover')r.task_passed=!!s.completed&&s.final_displacement_m<.25&&rateRms<.3;
   r.telemetry={channels,unique_published_estimates:selected.length,aligned_samples:errors.length,
     comparison:'unique estimate publications; time_boot_ms alignment is NOT IMU/GPS acquisition timing',
     current_position_rmse_m:Math.sqrt(mean(errors.map(e=>e[0]))),aligned_position_rmse_m:Math.sqrt(mean(errors.map(e=>e[1]))),
     current_velocity_rmse_m_s:Math.sqrt(mean(errors.map(e=>e[2]))),aligned_velocity_rmse_m_s:Math.sqrt(mean(errors.map(e=>e[3]))),
     max_age_s:Math.max(...frames.map(f=>f.sim_time_s-f.estimate.timestamp))};
   r.hover_rate_rms_after2s_rad_s=launch.mode==='hover'?rateRms:null;
   r.max_body_rate_rad_s=Math.max(...frames.map(f=>norm(f.truth.body_rate_flu)));
   if(c?.first_contact){const t=s.release_time_s+c.first_contact.time_s;
    const f=frames.reduce((a,b)=>Math.abs(a.sim_time_s-t)<Math.abs(b.sim_time_s-t)?a:b);
    const aligned=interpolate(frames,f.estimate.timestamp,'position_nwu');
    r.at_contact={age_s:f.sim_time_s-f.estimate.timestamp,estimate_minus_current_truth_m:f.estimate.position.map((x,i)=>x-f.truth.position_nwu[i]),
     estimate_minus_timestamp_aligned_truth_m:aligned?f.estimate.position.map((x,i)=>x-aligned[i]):null};
   }
  }
  results.push(r);
 }
 const report={passed:true,scope:'OFFLINE JSONL AUDIT COMPLETED; not an aggregate flight-success flag',
  limitations:['Single nominal run per setting, no reliability estimate','SITL GPS-backed EKF, not intended onboard navigation','Contact evaluator adaptation/reference diagnostics are shadow-controller values; not actor estimates'],results,input_sha256:hashes};
 await fs.writeFile(output+'/summary.json',JSON.stringify(report,null,2));
 await fs.copyFile(root+'/rm27/experiments/aggressive_contact/flight_control/audit.mjs',output+'/audit.mjs');
 await fs.writeFile(output+'/source_sha256.json',JSON.stringify({audit_mjs:createHash('sha256').update(await fs.readFile(output+'/audit.mjs')).digest('hex')},null,2));
 return report;
}
