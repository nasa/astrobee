classdef daqp< handle
    properties (SetAccess = immutable, Hidden = true)
        work_ptr;
        isdouble;
    end
    properties(SetAccess = protected)
        n = 0; m = 0; ms = 0
        H; f;
        A; bupper; blower; sense;
        bin_ids;
    end

    methods(Static)
        function [x,fval,exitflag,info] = quadprog(H,f,A,bupper,blower,sense)
            d = daqp();
            [exitflag,setup_time] = d.setup(H,f,A,bupper,blower,sense);
            if(exitflag <0)
                x = [];fval=[];info=[];
                return; 
            end
            [x,fval,exitflag,info] = d.solve();
            info.setup_time = setup_time;
        end

        function [x,fval,exitflag,info] = linprog(f,A,bupper,blower,sense)
            d = daqp();
            [exitflag,setup_time] = d.setup([],f,A,bupper,blower,sense);
            if(exitflag <0)
                x = [];fval=[];info=[];
                return; 
            end
            d.settings('eps_prox',1);
            d.settings('eta_prox',1e-6);
            [x,fval,exitflag,info] = d.solve();
            info.setup_time = setup_time;
        end
    end

    methods
        % Constructor (Create C workspace)
        function this = daqp(varargin)
            this.work_ptr= daqpmex('new', varargin{:});
            daqpmex('set_default_settings', this.work_ptr);
            this.isdouble = daqpmex('isdouble');
        end

        %% Destructor (Free C workspace) 
        function delete(this)
            daqpmex('delete', this.work_ptr);
        end

        function [x,fval,exitflag,info] = solve(this)
            [x,fval,exitflag,info] = daqpmex('solve', this.work_ptr,...
                this.H,this.f,this.A,this.bupper,this.blower,this.sense);
        end
        function [exitflag,setup_time] = setup(this,H,f,A,bupper,blower,sense)
            % TODO Check validity
            % TODO match double/single with c_float... 
            this.n = length(f);
            this.m = length(bupper);
            this.ms = this.m-size(A,1);
            if(this.isdouble)
                this.H = double(H);
                this.f = double(f);
                this.A = double(A'); % col.major => row.major 
                this.bupper = double(bupper);
                this.blower = double(blower);
            else
                this.H = single(H);
                this.f = single(f);
                this.A = single(A'); % col.major => row.major 
                this.bupper = single(bupper);
                this.blower = single(blower);
            end
            this.sense = int32(sense);
            this.bin_ids= int32(find(bitand(this.sense,16))-1);
            [exitflag,setup_time] = daqpmex('setup', this.work_ptr,this.H,this.f,this.A,this.bupper,this.blower,this.sense,this.bin_ids);
        end

        function settings = settings(this,varargin)
            settings = daqpmex('get_settings',this.work_ptr);
            if(length(varargin)==0) % No arguments => return current settings
                return
            end
            if(length(varargin)==1)
                if(isstruct(varargin{1})) % If struct as input treat it as new settings
                    new_settings = varargin{1};
                else 
                    if(isstr(varargin{1})&& strcmp(varargin{1},'default')) % set/return default settings
                        daqpmex('set_default_settings', this.work_ptr);
                        settings = daqpmex('get_settings', this.work_ptr);
                        return
                    else return % 1 arguments but not struct nor string
                    end
                end
            else
                new_settings = struct(varargin{:});
            end
            fn= fieldnames(new_settings);
            for k=1:numel(fn)
                if(isfield(settings,fn{k}) && isscalar(new_settings.(fn{k}))) 
                    settings = setfield(settings,fn{k},cast(new_settings.(fn{k}),class(settings.(fn{k}))));
                else
                    fprintf('Unable to set field %s\n',fn{k});
                    % Invalid field and/or value
                end
            end
            daqpmex('set_settings', this.work_ptr, settings);
        end

        function soften_constraints(this,ids)
            this.sense(ids) = this.sense(ids)+8;
            % TODO: update workspace
            this 
        end
        function update(this,H,f,A,bupper,blower,sense)
            update_mask = int32(0);
            if(size(H,1)==this.n && size(H,2) == this.n)
                this.H = H;
                update_mask = update_mask+1;
            end
            if(size(A,1)==(this.m-this.ms) && size(A,2) == this.n)
                this.A = A'; % col.major => row.major
                update_mask = update_mask+2;
            end
            if(length(f)==this.n)
                this.f = f;
                update_mask = update_mask+4;
            end
            if(length(bupper)==this.m && length(blower)==this.m)
                this.bupper = bupper;
                this.blower = blower;
                update_mask = update_mask+8;
            end
            if(length(sense)==this.m)
                this.sense = int32(sense);
                update_mask = update_mask+16;
            end
            daqpmex('update', this.work_ptr,...
                this.H,this.f,this.A,this.bupper,this.blower,this.sense,...
                update_mask);
        end
        function codegen(this,varargin)
            if(length(varargin)<1 || ~ischar(varargin{1}))
                fname='workspace';
            else
                fname=varargin{1};
            end
            if(length(varargin)<2 || ~ischar(varargin{2}))
                dir='';
            else
                dir=varargin{2};
            end
            daqpmex('codegen', this.work_ptr,fname,dir);
        end
    end
end
