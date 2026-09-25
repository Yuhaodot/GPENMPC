function report=run_passive_ground_dissipation_tests(outputDir)
% Test passive-ground functions.
arguments,outputDir (1,1) string,end
assert(~isfolder(outputDir)&&~isfile(outputDir),'test:OutputExists','No overwrite.');
build=fileparts(fileparts(mfilename('fullpath')));previousPath=path;
addpath(fullfile(build,'matlab_validation'));cleanup=onCleanup(@()path(previousPath)); %#ok<NASGU>
p=struct('damping_ratio',.90,'static_deflection_m',.035);J=diag([1.6 1.6 3]);
rows=struct('name',{},'passed',{},'detail',{});n=0;
for mass=[1 11.71 30]
    for mu=[.2 .4 .8]
        for sx=[-1 1]
            for sy=[-1 1]
                for magnitude=[0 1e-12 1e-4 .2 10 1e3]
                    v=magnitude*[sx;sy;-.7];w=magnitude*[sy;-sx;.3];
                    Fn=mass*9.80665;
                    [F,t,e]=m600check.passiveGroundDissipation(v,w,mass,J,Fn,mu,p);
                    expected=-mass*e.damping_rate_per_s*sum(v(1:2).^2)-e.damping_rate_per_s*(w.'*J*w);
                    ok=all(isfinite([F;t;e.power_W]))&&F(3)==0&&e.normal_force_delta_N==0 ...
                        &&e.power_W<=1e-11*max(1,abs(expected)) ...
                        &&e.combined_actual_N<=mu*Fn*(1+1e-13) ...
                        &&abs(e.power_W-e.scale*expected)<=1e-10*max(1,abs(expected));
                    record(sprintf('passive_cap_m%g_mu%g_s%d_%d_v%g',mass,mu,sx,sy,magnitude),ok,'world XY + body torque; power and common cone cap');
                end
            end
        end
    end
end
[F,t,e]=m600check.passiveGroundDissipation([2;-3;4],[.2;-.3;.1],11.71,J,0,.4,p);
record('no_contact_exact_zero',isequal(F,zeros(3,1))&&isequal(t,zeros(3,1))&&e.power_W==0,'Fn=0');
[F,t,e]=m600check.passiveGroundDissipation([2;-3;4],[.2;-.3;.1],11.71,J,100,0,p);
record('mu_zero_exact_zero',isequal(F,zeros(3,1))&&isequal(t,zeros(3,1))&&e.power_W==0,'mu=0 sensitivity edge, not selected candidate');
[F,t,e]=m600check.passiveGroundDissipation([0;0;4],zeros(3,1),11.71,J,100,.4,p);
record('pure_vertical_no_added_normal',isequal(F,zeros(3,1))&&isequal(t,zeros(3,1))&&e.scale==1,'vertical speed untouched');
v=[.2;-.3;.1];w=[.1;-.2;.3];S=diag([-1 -1 1]);
[F,t,e]=m600check.passiveGroundDissipation(v,w,11.71,J,114,.4,p);
[Fs,ts]=m600check.passiveGroundDissipation(S*v,S*w,11.71,S*J*S.',114,.4,p);
record('FRD_FLU_sign_transform',norm(Fs-S*F)<1e-12&&norm(ts-S*t)<1e-12,'signed body/world axis transform, no frame mixing');
record('derived_units',abs(e.damping_rate_per_s-2*.9*sqrt(9.80665/.035))<1e-12 ...
    &&abs(e.effective_gyration_radius_m-sqrt(1.6/11.71))<1e-12,'c in 1/s; gyration radius in m');
[F2,t2]=m600check.passiveGroundDissipation(v,w,23.42,2*J,228,.4,p);
record('mass_inertia_force_homogeneity',norm(F2-2*F)<1e-11&&norm(t2-2*t)<1e-11,'same acceleration with scaled mass/inertia/normal load');
[Fneg,tneg]=m600check.passiveGroundDissipation(-v,-w,11.71,J,114,.4,p);
record('all_axis_odd_symmetry',norm(Fneg+F)<1e-12&&norm(tneg+t)<1e-12,'reversed velocity reverses dissipation');
[~,~,small]=m600check.passiveGroundDissipation(v*1e-12,w*1e-12,11.71,J,114,.4,p);
record('near_zero_stable_limit',isfinite(small.scale)&&abs(small.scale-1)<1e-12,'no q=0 singularity');
bad(@()m600check.passiveGroundDissipation([NaN;0;0],w,11.71,J,114,.4,p),'velocity_nan','m600check:PassiveVelocity');
bad(@()m600check.passiveGroundDissipation(v.',w,11.71,J,114,.4,p),'velocity_shape','m600check:PassiveVelocity');
bad(@()m600check.passiveGroundDissipation(v,[Inf;0;0],11.71,J,114,.4,p),'omega_inf','m600check:PassiveOmega');
bad(@()m600check.passiveGroundDissipation(v,w,0,J,114,.4,p),'mass_zero','m600check:PassiveMass');
bad(@()m600check.passiveGroundDissipation(v,w,11.71,diag([1 0 3]),114,.4,p),'inertia_not_PD','m600check:PassiveInertia');
bad(@()m600check.passiveGroundDissipation(v,w,11.71,[1 1 0;0 2 0;0 0 3],114,.4,p),'inertia_nonsymmetric','m600check:PassiveInertia');
bad(@()m600check.passiveGroundDissipation(v,w,11.71,J,-1,.4,p),'normal_negative','m600check:PassiveNormalForce');
bad(@()m600check.passiveGroundDissipation(v,w,11.71,J,114,NaN,p),'mu_nan','m600check:PassiveMu');
bad(@()m600check.passiveGroundDissipation(v,w,11.71,J,114,.4,struct()),'missing_parameters','m600check:PassiveParameters');
pBad=p;pBad.static_deflection_m=0;
bad(@()m600check.passiveGroundDissipation(v,w,11.71,J,114,.4,pBad),'zero_deflection','m600check:PassiveParameters');
report=struct('schema','HOST_PASSIVE_GROUND_DISSIPATION_TEST_V1','passed',all([rows.passed]), ...
    'checks',numel(rows),'passed_checks',nnz([rows.passed]),'rows',rows,'hardware_actions',0,'COM_UDP_actions',0);
mkdir(outputDir);fid=fopen(fullfile(outputDir,'RESULT.json'),'w','n','UTF-8');assert(fid>=0,'test:File','Output unavailable.');
closer=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));clear closer
disp(jsonencode(struct('passed',report.passed,'checks',report.checks,'passed_checks',report.passed_checks)));
assert(report.passed,'test:Failed','Pure passive contact tests failed.');
    function record(name,pass,detail)
        n=n+1;rows(n)=struct('name',name,'passed',logical(pass),'detail',detail);
    end
    function bad(call,name,id)
        actual='';try,call();catch err,actual=err.identifier;end
        record(name,strcmp(actual,id),actual);
    end
end
