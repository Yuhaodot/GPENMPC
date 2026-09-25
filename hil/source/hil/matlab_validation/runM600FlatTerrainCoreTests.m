function report=runM600FlatTerrainCoreTests(parameterMat,outputJson)
%RUNM600FLATTERRAINCORETESTS Pure MATLAB terrain-wrapper numerical fixtures.
% Run once in a fresh MATLAB process for the first-call negative fixture.
% Exercises the terrain wrapper with numerical fixtures in MATLAB.
assert(isfile(parameterMat)&&~isfile(outputJson),'m600check:TerrainTestPaths','Exact inputs/new output required.');
here=fileparts(mfilename('fullpath'));addpath(here);m600check.loadFixture();
loaded=load(parameterMat,'parameters','environment');p=loaded.parameters;env=loaded.environment;
checks=struct('name',{},'passed',{},'maximum_absolute_difference',{});
u=zeros(16,1);angles=[.08;-.04;.2];ground=-8.050000190734863;
% Cold-start tests precede ALL wrapper calls. clear packageFunction inside an
% executing compiled test is not a reliable persistent-state reset mechanism.
coldTerrain=zeros(15,1);coldTerrain(1)=ground;
coldWorld=[0;0;ground];coldEnv=env;coldEnv.reference_jet_ned(3)=ground;
[~,cold]=m600check.copterSimFlatTerrainCore(u,false,coldWorld,zeros(3,1),coldEnv,coldTerrain,p);
assert(cold.failure_code==4&&cold.terrain_failure_reason==1&&~cold.state_snapshot_available&&~cold.terrain_locked);
record('cold_call_requires_explicit_reset_no_hidden_ground',0);
coldBadTerrain=coldTerrain;coldBadTerrain(1)=NaN;
[~,coldBad]=m600check.copterSimFlatTerrainCore(u,true,coldWorld,zeros(3,1),coldEnv,coldBadTerrain,p);
% Once the no-reset first fault is latched, an invalid reset cannot clear or
% overwrite it. It still cannot invent an accepted aircraft-state snapshot.
assert(coldBad.failure_code==4&&coldBad.terrain_failure_reason==1&&~coldBad.state_snapshot_available);
record('invalid_reset_does_not_clear_cold_fault_or_invent_state',0);
% Every sequence compares all y fields, not just z/velocity. Position is the
% only translated field; vector/rotation/force/derivative semantics persist.
for altitude=[0,2]
    base=coreSequence(0,altitude,false);
    zero=coreSequence(0,altitude,true);
    translated=coreSequence(ground,altitude,true);
    worst=0;
    for k=1:numel(base)
        worst=max(worst,compareY(base(k).y,zero(k).y,0));
        worst=max(worst,compareY(base(k).y,translated(k).y,ground));
        near(base(k).d.state_up,translated(k).d.state_up);
        assert(base(k).d.sim_time_s==translated(k).d.sim_time_s);
        assert(base(k).d.ground_confirmed==translated(k).d.ground_confirmed);
    end
    record(sprintf('all_fields_ground_translation_altitude_%g',altitude),worst);
end
terrain=zeros(15,1);terrain(1)=ground;
world=[0;0;ground];worldEnv=env;worldEnv.reference_jet_ned(3)=ground;
[y,d]=m600check.copterSimFlatTerrainCore(u,true,world,zeros(3,1),worldEnv,terrain,p);
assert(d.terrain_locked&&d.state_snapshot_available&&d.observation_valid);
assert(d.ground_confirmed&&d.contact_active&&d.local_ground_relative_position_ned_m(3)==0);
record('actual_grasslands_spawn_is_on_independent_terrain',near(y.position_ned_m,world));
world(3)=ground-3;
[y,d]=m600check.copterSimFlatTerrainCore(u,true,world,zeros(3,1),worldEnv,terrain,p);
assert(~d.ground_confirmed&&~d.contact_active);
record('airborne_initial_z_never_becomes_ground',near(d.local_ground_relative_position_ned_m,[0;0;-3]));
record('free_fall_specific_force_unchanged',near(y.specific_force_body_frd_mps2,zeros(3,1)));
[accepted,da]=m600check.copterSimFlatTerrainCore(u,false,world,zeros(3,1),worldEnv,terrain,p);
assert(da.step_accepted&&da.sim_time_s==.01);
bad=terrain;bad(1)=NaN;
[failed,df]=m600check.copterSimFlatTerrainCore(u,false,world,zeros(3,1),worldEnv,bad,p);
assert(df.failed&&df.failure_code==4&&df.terrain_failure_reason==2&&~df.observation_valid&&~df.step_accepted);
assert(df.sim_time_s==da.sim_time_s&&df.plant_step_count==da.plant_step_count);
record('later_nonfinite_terrain_freezes_last_accepted_state',compareY(accepted,failed,0));
u(1:6)=.9;
[latched,dl]=m600check.copterSimFlatTerrainCore(u,false,world,zeros(3,1),worldEnv,terrain,p);
assert(dl.failure_code==4&&dl.sim_time_s==da.sim_time_s&&dl.terrain_failure_reason==2);
record('good_terrain_does_not_clear_fault_or_advance',compareY(accepted,latched,0));
u(:)=0;
[~,fresh]=m600check.copterSimFlatTerrainCore(u,true,world,zeros(3,1),worldEnv,terrain,p);
assert(~fresh.failed&&fresh.reset_applied&&fresh.sim_time_s==0&&fresh.plant_step_count==0);
record('explicit_new_run_reset_clears_terrain_fault',0);
changed=terrain;changed(1)=double(single(ground)+eps(single(ground)));
[air,dc]=m600check.copterSimFlatTerrainCore(u,false,world,zeros(3,1),worldEnv,changed,p);
assert(~dc.failed&&dc.step_accepted&&dc.sim_time_s==.01&&dc.terrain_world_ned_z_m==ground);
assert(~dc.contact_active&&air.position_ned_m(3)<ground-2.99);
record('one_float_ulp_preserves_airborne_state',0);
% Same airborne command/state with a genuinely different surface must retain
% world motion while clear of both surfaces; the contact input is not a pose.
changed(1)=ground-.1;
[~,~]=m600check.copterSimFlatTerrainCore(u,true,world,zeros(3,1),worldEnv,terrain,p);
[air2,dc2]=m600check.copterSimFlatTerrainCore(u,false,world,zeros(3,1),worldEnv,changed,p);
assert(~dc2.failed&&~dc2.contact_active&&dc2.terrain_world_ned_z_m==ground);
record('changing_surface_does_not_move_world_pose_or_reference',compareY(air,air2,0));
% Surface height affects the same contact law, rather than being ignored.
contactState=zeros(19,1);contactState(7)=1;
mass=p.profile.mass_properties.base_mass_kg+env.payload_kg+p.mission.plant_mismatch.mass_bias_kg;
c0=m600check.contactKernel(contactState,mass,p.contact);
pc=p.contact;pc.surface_height_up_m=.001;c1=m600check.contactKernel(contactState,mass,pc);
assert(c1.contact_force_n>c0.contact_force_n&&c1.contact_deflection_m>c0.contact_deflection_m);
record('sampled_surface_changes_contact_not_aircraft_state',0);
newWorld=[0;0;changed(1)];newEnv=worldEnv;newEnv.reference_jet_ned(3)=changed(1);
[yn,dn]=m600check.copterSimFlatTerrainCore(u,true,newWorld,zeros(3,1),newEnv,changed,p);
assert(~dn.failed&&dn.terrain_world_ned_z_m==changed(1)&&dn.ground_confirmed);
record('only_explicit_reset_can_lock_new_height',near(yn.position_ned_m,newWorld));
bad=changed;bad(15)=Inf;
[~,du]=m600check.copterSimFlatTerrainCore(u,false,newWorld,zeros(3,1),newEnv,bad,p);
assert(du.failure_code==4&&du.terrain_failure_reason==2);
record('nonfinite_terrain_vector_rejected',0);
[~,~]=m600check.copterSimFlatTerrainCore(u,true,newWorld,zeros(3,1),newEnv,changed,p);
u(1)=NaN;
[~,coreBad]=m600check.copterSimFlatTerrainCore(u,false,newWorld,zeros(3,1),newEnv,changed,p);
assert(coreBad.failed&&coreBad.failure_code==1&&coreBad.core_failure_code==1&&~coreBad.terrain_fault_latched);
record('existing_control_fault_retains_core_identity',0);
bad(1)=NaN;
[~,both]=m600check.copterSimFlatTerrainCore(u,false,newWorld,zeros(3,1),newEnv,bad,p);
assert(both.failure_code==4&&both.core_failure_code==1);
record('terrain_failure_does_not_erase_prior_core_failure_code',0);
report=struct('status','PASS_HOST_ONLY_FIXED_DATUM_SAMPLED_GROUND_AND_LATCH', ...
    'test_count',numel(checks),'passed_count',nnz([checks.passed]),'checks',checks, ...
    'maximum_absolute_difference',max([checks.maximum_absolute_difference]), ...
    'grasslands_fixture_world_ground_ned_z_m',ground, ...
    'terrain_source','EXPLICIT_TERRAININ15D_FIRST_ELEMENT_NOT_AIRCRAFT_INITIAL_Z', ...
    'parameter_path',string(parameterMat),'parameter_sha256',gpenmpcNative.fileSha256(parameterMat), ...
    'core_sha256',gpenmpcNative.fileSha256(which('m600check.copterSimIoCore')), ...
    'wrapper_sha256',gpenmpcNative.fileSha256(which('m600check.copterSimFlatTerrainCore')), ...
    'hardware_actions',0,'socket_open',0,'simulator_runs',0);
f=fopen(outputJson,'w','n','UTF-8');assert(f>=0);closer=onCleanup(@()fclose(f)); %#ok<NASGU>
fprintf(f,'%s\n',jsonencode(report,PrettyPrint=true));disp(jsonencode(report,PrettyPrint=true));

    function sequence=coreSequence(z,altitude,useWrapper)
        sequence=cell(21,1);position=[1;2;z-altitude];environment=env;
        environment.reference_jet_ned(3)=z-1;
        groundInput=zeros(15,1);groundInput(1)=z;
        for j=0:20
            command=zeros(16,1);command(1:6)=.25+.02*sin(j*.3);
            if useWrapper
                [output,diag]=m600check.copterSimFlatTerrainCore(command,j==0,position,angles,environment,groundInput,p);
            else
                [output,diag]=m600check.copterSimIoCore(command,j==0,position,angles,environment,p);
            end
            assert(~diag.failed);
            sequence{j+1}=struct('y',output,'d',diag);
        end
        sequence=vertcat(sequence{:});
    end
    function record(name,difference)
        checks(end+1)=struct('name',name,'passed',true,'maximum_absolute_difference',difference); %#ok<AGROW>
    end
end

function maximum=compareY(local,world,terrainZ)
expected=local;expected.position_ned_m(3)=expected.position_ned_m(3)+terrainZ;
names=fieldnames(expected);maximum=0;
for i=1:numel(names),name=names{i};maximum=max(maximum,near(expected.(name),world.(name)));end
end
function difference=near(a,b)
assert(isequal(size(a),size(b))&&all(isfinite(a(:)))&&all(isfinite(b(:))));
difference=max(abs(double(a(:))-double(b(:))));
assert(difference<=1e-9,'m600check:TerrainArithmeticMismatch','%.17g',difference);
end
