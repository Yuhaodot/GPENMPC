classdef ReferenceEncoderOracle < gpenmpcNative.MavlinkSerialLink
    % HOST-only interceptor: run the unchanged parent's sendSetpoint method
    % but capture sendMessage instead of opening any transport.
    properties
        Captured = struct()
    end
    methods
        function sendMessage(obj,name,payload)
            obj.Captured=struct('message_name',string(name),'Payload',payload);
        end
        function varargout=open(~,varargin) %#ok<STOUT,INUSD>
            error('gpenmpcTaskIo:OracleNoCOM','This test oracle cannot open COM.');
        end
        function delete(obj)
            obj.close();
            try,delete(obj.Serializer);catch,end
        end
    end
end
