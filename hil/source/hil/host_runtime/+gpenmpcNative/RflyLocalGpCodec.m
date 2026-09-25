classdef RflyLocalGpCodec
    % Encode and decode RGP1/RGR1 messages.
    methods (Static)
        function q=decodeRequest(bytes)
            % A request is checked repeatedly by the predictor and sender.
            % Reuse only a successful parse of identical complete bytes;
            % age, session and replay checks remain in those callers.
            persistent previousBytes previousRequest
            assert(isa(bytes,'uint8')&&numel(bytes)==310,'gpenmpcNative:LocalGpShape','Exact byte count required.');
            bytes=bytes(:);
            if isequal(bytes,previousBytes),q=previousRequest;return,end
            bytes=checked(bytes,310,'RGP1');
            counters=read(bytes,23:62,'uint64');
            q=struct('identity',identity(bytes),'source_timestamp_ns',counters(1), ...
                'source_generation',counters(2),'output_generation',counters(3), ...
                'original_publication_us',counters(4), ...
                'publication_valid_until_us',counters(5), ...
                'configuration_sha256',bytes(63:94),'gp_model_sha256',bytes(95:126), ...
                'request19',read(bytes,127:278,'double'),'original_bytes',bytes, ...
                'original_request_sha256',digest(bytes));
            [config,model]=gpenmpcNative.RflyLocalGpCodec.identities();
            assert(q.source_timestamp_ns>0&&q.source_generation>0&&q.output_generation>0 ...
                &&q.original_publication_us>0 ...
                &&q.original_publication_us>=idivide(q.source_timestamp_ns,uint64(1000)) ...
                &&q.publication_valid_until_us>=q.original_publication_us ...
                &&isequal(q.configuration_sha256,config)&&isequal(q.gp_model_sha256,model) ...
                &&all(isfinite(q.request19))&&q.request19(1)==1, ...
                'gpenmpcNative:LocalGpRequest','Invalid canonical numerical request.');
            previousBytes=bytes;previousRequest=q;
        end
        function bytes=encodeReply(q,result18)
            % Re-decode the exact request bytes, not mutable caller fields.
            q=gpenmpcNative.RflyLocalGpCodec.decodeRequest(q.original_bytes);
            assert(isa(result18,'double')&&numel(result18)==18&&all(isfinite(result18)), ...
                'gpenmpcNative:LocalGpReply','All eighteen original numerical fields required.');
            bytes=[uint8('RGR1').';q.original_bytes(5:46);q.original_request_sha256; ...
                q.gp_model_sha256;write(result18(:))];
            assert(numel(bytes)==254);bytes=[bytes;digest(bytes)];
            % hard-invalid is original field15 (C index14), not transport loss.
        end
        function r=decodeReply(bytes)
            persistent previousBytes previousReply
            assert(isa(bytes,'uint8')&&numel(bytes)==286,'gpenmpcNative:LocalGpShape','Exact byte count required.');
            bytes=bytes(:);
            if isequal(bytes,previousBytes),r=previousReply;return,end
            bytes=checked(bytes,286,'RGR1');
            counters=read(bytes,23:46,'uint64');
            r=struct('identity',identity(bytes),'source_timestamp_ns',counters(1), ...
                'source_generation',counters(2),'output_generation',counters(3), ...
                'original_request_sha256',bytes(47:78),'gp_model_sha256',bytes(79:110), ...
                'result18',read(bytes,111:254,'double'),'original_bytes',bytes);
            [~,model]=gpenmpcNative.RflyLocalGpCodec.identities();
            assert(r.source_timestamp_ns>0&&r.source_generation>0&&r.output_generation>0 ...
                &&any(r.original_request_sha256~=0)&&isequal(r.gp_model_sha256,model) ...
                &&all(isfinite(r.result18)),'gpenmpcNative:LocalGpReply','Invalid original numerical reply.');
            previousBytes=bytes;previousReply=r;
        end
        function fragments=replyFragments(bytes,targetSystem,targetComponent)
            r=gpenmpcNative.RflyLocalGpCodec.decodeReply(bytes);bytes=r.original_bytes;
            assert(isa(targetSystem,'uint8')&&isscalar(targetSystem)&&targetSystem>0 ...
                &&isa(targetComponent,'uint8')&&isscalar(targetComponent)&&targetComponent>0);
            fragments=cell(3,1);
            generationBytes=write(r.output_generation);
            for k=0:2
                n=min(119,numel(bytes)-119*k);payload=zeros(128,1,'uint8');
                payload(1)=uint8(144+k);payload(2:9)=generationBytes;
                payload(10:9+n)=bytes(119*k+1:119*k+n);
                fragments{k+1}=struct('target_system',targetSystem,'target_component',targetComponent, ...
                    'payload_type',uint16(42002),'payload_length',uint8(9+n),'payload',payload);
            end
        end
        function [config,model]=identities()
            persistent configBytes modelBytes
            if isempty(configBytes)
                configBytes=unhex('A859433D0AA774013341444A4B9AE971A34F12B4FB89C0A004B2CB3E013FCEBA');
                modelBytes=unhex('4A09E9A3D4818B5555CD3439A6D2133026EEA0FDC2774A1FB17F9E05486E5BB2');
            end
            config=configBytes;model=modelBytes;
        end
    end
end
function bytes=checked(bytes,n,magic)
assert(isa(bytes,'uint8')&&numel(bytes)==n,'gpenmpcNative:LocalGpShape','Exact byte count required.');
bytes=bytes(:);assert(isequal(bytes(1:4),uint8(magic).')&&isequal(digest(bytes(1:end-32)),bytes(end-31:end)), ...
    'gpenmpcNative:LocalGpDigest','Magic or checksum mismatch.');
end
function i=identity(bytes)
ids=read(bytes,5:20,'uint64');
i=struct('uid',ids(1),'boot_generation',ids(2), ...
    'system',bytes(21),'component',bytes(22));
assert(i.uid>0&&i.boot_generation>0&&i.system>0&&i.component>0, ...
    'gpenmpcNative:LocalGpIdentity','Exact nonzero identity required.');
end
function value=read(bytes,indices,kind)
value=typecast(bytes(indices),kind);[~,~,endian]=computer;
if endian=='L',value=swapbytes(value);end
value=value(:);
end
function bytes=write(value)
[~,~,endian]=computer;if endian=='L',value=swapbytes(value);end
bytes=reshape(typecast(value(:),'uint8'),[],1);
end
function h=digest(bytes)
md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(bytes(:),'int8'));
h=reshape(typecast(md.digest(),'uint8'),[],1);
end
function b=unhex(s),b=uint8(sscanf(s,'%2x'));end
