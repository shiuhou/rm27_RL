"""Separate task inputs from independent physical rejection in a saved runner."""


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise ValueError('Expected frozen causal-flight context changed: '+old[:70])
    return text.replace(old, new)


def prepare_source(text):
    old="""    (output/'metadata.json').write_text(json.dumps(metadata,indent=2))"""
    new="""    if hasattr(plant,'physical_fixture_evidence'):
        metadata['physical_fixture']=plant.physical_fixture_evidence
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2))"""
    text=replace_once(text,old,new)
    old="""    yaw=math.atan2(plant.data.xmat[plant.body].reshape(3,3)[1,0],plant.data.xmat[plant.body].reshape(3,3)[0,0])"""
    new="""    physical_violations=[]
    yaw=(float(route.mission['surveyed_initial_yaw_nwu_rad']) if telemetry_guidance and route else
        math.atan2(plant.data.xmat[plant.body].reshape(3,3)[1,0],plant.data.xmat[plant.body].reshape(3,3)[0,0]))
    if telemetry_guidance and not route:
        raise ValueError('Explicit surveyed-yaw route required for causal development')
    summary['actor_input_policy']='PX4 telemetry and current image observations; physical monitors score after actor completion'"""
    text=replace_once(text,old,new)
    old="""                    if t-phase_start>15:raise RuntimeError('AprilTag reacquisition timed out')"""
    new="""                    maximum_gap=route.mission.get('maximum_return_image_gap_s',15.) if route else 15.
                    image_gap=(t-visual_guard.source_time if visual_guard.source_time is not None else float('inf'))
                    if t-phase_start>15 or (not visual_guard.fresh(t) and image_gap>maximum_gap):
                        raise RuntimeError('AprilTag reacquisition timed out')"""
    text=replace_once(text,old,new)
    text=replace_once(text,"""                else:
                    local=physical_local;speed=physical_speed""","""                elif telemetry_guidance:
                    # Uninitialized neutral values cannot guide an armed
                    # vehicle; never substitute simulator truth for telemetry.
                    if tel.get('armed'):raise RuntimeError('No PX4 local navigation while armed')
                    local=np.zeros(3);speed=0.
                else:
                    local=physical_local;speed=physical_speed""")
    old="""                if tilt>math.radians(route.maximum_tilt_deg if phase=='ROUTE' else 25) or speed>(route.maximum_speed_m_s if phase=='ROUTE' else 2) or (phase!='ROUTE' and (local[2]>1.1 or np.linalg.norm(local[:2])>.5)):
                    raise RuntimeError(f'Flight envelope exceeded: position={local}, speed={speed}, tilt={math.degrees(tilt)}')"""
    new="""                mission_tilt=tilt
                if telemetry_guidance:
                    q=np.asarray(tel.get('quaternion',[]),float)
                    valid_attitude=(q.shape==(4,) and np.isfinite(q).all() and abs(np.linalg.norm(q)-1)<=.02 and
                        0<=t-tel.get('attitude_time',-100)<=.3)
                    if not valid_attitude and phase!='INITIALIZE':raise RuntimeError('No fresh valid PX4 attitude')
                    mission_tilt=(math.acos(float(np.clip(1-2*(q[1]**2+q[2]**2),-1,1))) if valid_attitude else 0.)
                if mission_tilt>math.radians(route.maximum_tilt_deg if phase=='ROUTE' else 25) or speed>(route.maximum_speed_m_s if phase=='ROUTE' else 2) or (phase!='ROUTE' and (local[2]>1.1 or np.linalg.norm(local[:2])>.5)):
                    raise RuntimeError(f'Flight envelope exceeded: position={local}, speed={speed}, tilt={math.degrees(mission_tilt)}')"""
    text=replace_once(text,old,new)
    text=replace_once(text,"""if np.linalg.norm(plant.data.qpos[qa:qa+2]-initial[:2])>.001 or abs(plant.data.qpos[qa+2]-initial[2])>.002:raise RuntimeError('Inactive vehicle moved')""",
        """if np.linalg.norm(plant.data.qpos[qa:qa+2]-initial[:2])>.001 or abs(plant.data.qpos[qa+2]-initial[2])>.002:physical_violations.append(dict(time_s=t,reason='Inactive vehicle moved'))""")
    text=replace_once(text,"""if t>1 and np.max(np.abs(plant.data.qvel[va:va+6]))>.001:raise RuntimeError('Inactive vehicle not resting')""",
        """if t>1 and np.max(np.abs(plant.data.qvel[va:va+6]))>.001:physical_violations.append(dict(time_s=t,reason='Inactive vehicle not resting'))""")
    text=replace_once(text,"""if 1-2*(q[1]**2+q[2]**2)<math.cos(math.radians(1)):raise RuntimeError('Inactive vehicle tilted')""",
        """if 1-2*(q[1]**2+q[2]**2)<math.cos(math.radians(1)):physical_violations.append(dict(time_s=t,reason='Inactive vehicle tilted'))""")
    text=replace_once(text,"""if np.any(plant.data.ctrl[inactive]) or np.any(plant.data.actuator_force[inactive]):raise RuntimeError('Inactive motor received thrust')""",
        """if np.any(plant.data.ctrl[inactive]) or np.any(plant.data.actuator_force[inactive]):physical_violations.append(dict(time_s=t,reason='Inactive motor received thrust'))""")
    text=replace_once(text,"""if 'rm27_dock_collision_bottom' not in c['geoms'] and not (phase=='ROUTE' and route.contact_allowed(c)):raise RuntimeError('Unintended contact: '+str(c['geoms']))""",
        """if 'rm27_dock_collision_bottom' not in c['geoms'] and not (phase=='ROUTE' and route.contact_allowed(c)):physical_violations.append(dict(time_s=t,reason='Unintended contact',contact=c))""")
    text=replace_once(text,"""if 'rm27_dock_collision_bottom' in c['geoms'] and (c['force_N']>10 or c['distance_m']<-.005):raise RuntimeError('Unstable landing contact')""",
        """if 'rm27_dock_collision_bottom' in c['geoms'] and (c['force_N']>10 or c['distance_m']<-.005):physical_violations.append(dict(time_s=t,reason='Unstable landing contact',contact=c))""")
    old="""        summary.update(phase=phase,physics_time_s=float(plant.data.time),landing=landing,"""
    new="""        summary['independent_physical_monitor']=dict(passed=not physical_violations,violations=physical_violations,
            source='independent physics scoring only; no actor feedback')
        if physical_violations:
            summary['passed']=False
            summary['failure']=summary.get('failure') or 'Independent physical monitor rejected task'
        summary.update(phase=phase,physics_time_s=float(plant.data.time),landing=landing,"""
    return replace_once(text,old,new)
