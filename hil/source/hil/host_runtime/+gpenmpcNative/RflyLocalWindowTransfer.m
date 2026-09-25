classdef RflyLocalWindowTransfer < handle
    % Transfer RWW1 fragments through the IO owner.
    % Confirm installation through RLC feedback.
    properties (SetAccess=private)
        Failed=false
        Failure=''
    end
    properties (Access=private)
        Registered
        MaximumAgeNs
        LastGeneration=uint64(0)
        Identity=[]
        Packets={}
        Next=1
        Outstanding=false
        Submitted=zeros(244,1,'uint64')
        Returned=zeros(244,1,'uint64')
        FirstSubmit=uint64(0)
        Expiry=uint64(0)
        Attempts=0
        ReturnedCount=0
    end
    methods
        function self=RflyLocalWindowTransfer(registered,maximumAgeNs)
            % Same 1 s bound as actual CanonicalLocalSessionEntry assembler.
            % This is an engineering transport bound, not measured link WCET.
            assert(isa(maximumAgeNs,'uint64')&&isscalar(maximumAgeNs)&&maximumAgeNs>0 ...
                &&maximumAgeNs<=uint64(1000000000),'gpenmpcNative:LocalWindowTransportBound');
            self.Registered=registered;self.MaximumAgeNs=maximumAgeNs;
        end
        function receipt=begin(self,window,serviceIo)
            if nargin<3,serviceIo=@() [];end
            try
                assert(~self.Failed&&~self.Outstanding&&(isempty(self.Identity)||self.Next==245), ...
                    'gpenmpcNative:LocalWindowInFlight','Do not replace/restart a pending or failed window.');
                assert(window.window_generation>self.LastGeneration,'gpenmpcNative:LocalWindowReplay');
                [~,packets,id]=gpenmpcNative.RflyLocalWindowCodec.encode(window,self.Registered,serviceIo);
                self.Identity=id;self.Packets=packets;self.LastGeneration=window.window_generation;
                self.Next=1;self.Submitted(:)=0;self.Returned(:)=0;self.FirstSubmit=uint64(0);self.Expiry=uint64(0);
                self.Attempts=0;self.ReturnedCount=0;receipt=self.evidence();
            catch ex,self.fail(ex.identifier);rethrow(ex);end
        end
        function payload=frame(self)
            assert(~self.Failed&&~isempty(self.Identity)&&self.Next<=244&&~self.Outstanding, ...
                'gpenmpcNative:LocalWindowUnavailable');payload=self.Packets{self.Next};
        end
        function binding=attempt(self,nowNs)
            try
                self.frame();assert(isa(nowNs,'uint64')&&isscalar(nowNs)&&nowNs>0);
                if self.FirstSubmit==0
                    assert(nowNs<=intmax('uint64')-self.MaximumAgeNs);
                    self.FirstSubmit=nowNs;self.Expiry=nowNs+self.MaximumAgeNs;
                end
                assert(nowNs>=self.FirstSubmit&&nowNs<=self.Expiry ...
                    &&(self.Next==1||nowNs>=self.Returned(self.Next-1)), ...
                    'gpenmpcNative:LocalWindowExpired','No fragment may renew the first-send lifetime.');
                self.Submitted(self.Next)=nowNs;self.Attempts=self.Attempts+1;self.Outstanding=true;
                binding=struct('window',self.Identity,'fragment_index',self.Next, ...
                    'first_original_submit_ns',self.FirstSubmit,'original_submit_ns',nowNs,'valid_until_host_ns',self.Expiry);
            catch ex,self.fail(ex.identifier);rethrow(ex);end
        end
        function sent(self,nowNs)
            try
                assert(~self.Failed&&self.Outstanding&&isa(nowNs,'uint64')&&isscalar(nowNs));
                self.Returned(self.Next)=nowNs;self.ReturnedCount=self.ReturnedCount+1;self.Outstanding=false;
                assert(nowNs>=self.Submitted(self.Next)&&nowNs<=self.Expiry, ...
                    'gpenmpcNative:LocalWindowExpired','Late actual return stays recorded and poisons this transfer.');
                self.Next=self.Next+1;
            catch ex,self.fail(ex.identifier);rethrow(ex);end
        end
        function fail(self,reason)
            if ~self.Failed,self.Failed=true;self.Failure=char(reason);end
        end
        function e=evidence(self)
            e=struct('schema','EXISTING_RWW1_BOUNDED_TRANSFER_V1','identity',self.Identity, ...
                'first_original_submit_ns',self.FirstSubmit,'valid_until_host_ns',self.Expiry, ...
                'maximum_transport_age_ns',self.MaximumAgeNs,'next_fragment',self.Next, ...
                'attempted_count',self.Attempts,'send_returned_count',self.ReturnedCount, ...
                'submitted_ns',self.Submitted,'returned_ns',self.Returned, ...
                'outstanding_send',self.Outstanding,'send_complete',self.Next==245&&~self.Failed, ...
                'failed',self.Failed,'failure',self.Failure,'board_receipt_proven',false,'control_authority',false);
        end
    end
end
