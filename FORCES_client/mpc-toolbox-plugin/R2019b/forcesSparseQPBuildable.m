classdef forcesSparseQPBuildable < coder.ExternalDependency
%% Interface between lineat MPC and FORCESPRO QP Solver (internal)

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.

    methods (Static)
        
        function name = getDescriptiveName(~)
            name = 'forcesSparseBuildable';
        end
        
        function b = isSupportedContext(context)
            b = context.isMatlabHostTarget();
        end        
        
        function updateBuildInfo(buildInfo, buildConfig)
            coder.allowpcode('plain');
            % FORCESPRO path
            forcespath = fileparts(which('FORCESversion'));
            % solver name
            solvername = 'customForcesSparseQP';
            % solver header
            headerPath = ['$(START_DIR)' filesep solvername filesep 'include'];
			isTargetPlatform = false;
            isDSpaceMABII = strcmpi(buildConfig.getConfigProp('SystemTargetFile'), 'rti1401.tlc');
            isDSpaceMABXIII = strcmpi(buildConfig.getConfigProp('SystemTargetFile'), 'dsrt.tlc');
            isSpeedGoat = strcmpi(buildConfig.getConfigProp('SystemTargetFile'), 'slrt.tlc');
            isSpeedGoatQNX = strcmpi(buildConfig.getConfigProp('SystemTargetFile'), 'slrealtime.tlc');
            isTargetPlatform = isTargetPlatform || isDSpaceMABII || isDSpaceMABXIII || isSpeedGoat || isSpeedGoatQNX;
            buildInfo.addIncludePaths(headerPath);
            % solver library
            libPriority = '';
            libPreCompiled = true;
            libLinkOnly = true;
            try 
                thisCompiler = mex.getCompilerConfigurations('C','Selected');
                settings.mexcomp.name = thisCompiler(1).Name;
                settings.mexcomp.ver = thisCompiler(1).Version;
                settings.mexcomp.vendor = thisCompiler(1).Manufacturer;
            catch
                settings.mexcomp = [];
            end
            if(isfield(settings, 'mexcomp') && isstruct(settings.mexcomp) && strncmpi(settings.mexcomp.name, 'MinGW', 5))
                isMinGW = true;
            else
                isMinGW = false;
            end
            
			if (isDSpaceMABII || isSpeedGoat)
                libName = [solvername '.lib'];
            elseif( ismac || isunix || isMinGW || isDSpaceMABXIII || isSpeedGoatQNX)
                libName = ['lib' solvername '.a'];
            else
                libName = [solvername '_static.lib'];  
            end
            if(isTargetPlatform)
                libPath = ['$(START_DIR)' filesep solvername filesep 'lib_target'];
            else
                libPath = ['$(START_DIR)' filesep solvername filesep 'lib'];
            end
            buildInfo.addLinkObjects(libName, libPath, libPriority, libPreCompiled, libLinkOnly);
            % additional standard library
            if ispc && ~isTargetPlatform && ~isMinGW
                libPathExtra = [forcespath filesep 'libs_Intel' filesep 'win64'];
                buildInfo.addLinkObjects('libmmt.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('libirc.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('svml_dispmt.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('libdecimal.lib', libPathExtra, libPriority, libPreCompiled, libLinkOnly);
                buildInfo.addLinkObjects('iphlpapi.lib', ['$(MATLAB_ROOT)' filesep 'sys' filesep 'lcc64' filesep 'lcc64' filesep 'lib64'], libPriority, libPreCompiled, libLinkOnly);
            else
                if(isMinGW)
                    buildInfo.addLinkObjects('iphlpapi.lib', ['$(MATLAB_ROOT)' filesep 'sys' filesep 'lcc64' filesep 'lcc64' filesep 'lib64'], libPriority, libPreCompiled, libLinkOnly);
                else
                    buildInfo.addLinkObjects('-lm', '', libPriority, libPreCompiled, libLinkOnly);
                end
            end
        end
        
        function [output,exitflag,SolverInfo] = forcesSparseQP(data,onlinedata,x,yref,uref,md,trueLastMV)
            params = forcesSparseGetParamValues(data,onlinedata,x,yref,uref,md,trueLastMV);
            FORCES_SolverName = 'customForcesSparseQP';
            headerName = [FORCES_SolverName '.h'];
            coder.cinclude(headerName);
            % inputs based on C interface defined in the header file
            coder.cstructname(params,[FORCES_SolverName '_params'],'extern','HeaderFile',headerName);
            output = struct('DecisionVariables', zeros(data.fLength(end),1));
            coder.cstructname(output,[FORCES_SolverName '_output'],'extern','HeaderFile',headerName);
            SolverInfo = struct('it', int32(0), 'it2opt', int32(0), 'res_eq', 0, 'res_ineq', 0, 'pobj', 0, 'dobj', 0, 'dgap', 0, 'rdgap', 0, 'mu', 0, 'mu_aff', 0, 'sigma', 0, 'lsit_aff', int32(0), 'lsit_cc', int32(0), 'step_aff', 0, 'step_cc', 0, 'solvetime', 0);
            coder.cstructname(SolverInfo,[FORCES_SolverName '_info'],'extern','HeaderFile',headerName);
            FILE = coder.opaque('FILE *','NULL','HeaderFile',headerName);
            exitflag = int32(0); %#ok<NASGU>
            % generate code with solver DLL/LIB
            exitflag = coder.ceval([FORCES_SolverName '_solve'],coder.ref(params),coder.ref(output),coder.ref(SolverInfo),FILE);
        end
        
    end
end

