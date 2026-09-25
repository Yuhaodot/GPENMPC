function value=rflyOriginalHostMonotonicNs()
% Convert Windows Stopwatch ticks using integer arithmetic.
persistent frequency billion maximumSeconds
if isempty(frequency)
 frequency=uint64(System.Diagnostics.Stopwatch.Frequency);
 assert(frequency>0,'gpenmpcNative:HostClockFrequency','Invalid platform counter frequency.');
 billion=uint64(1000000000);maximumSeconds=idivide(intmax('uint64'),billion);
end
% Frequency and unit/overflow constants are fixed for this process. Only the
% real counter is sampled on every call; no timestamp is cached or renewed.
ticks=uint64(System.Diagnostics.Stopwatch.GetTimestamp());
seconds=idivide(ticks,frequency);remainder=rem(ticks,frequency);
assert(seconds<=maximumSeconds&&remainder<=maximumSeconds, ...
    'gpenmpcNative:HostClockOverflow','Original counter cannot be represented in uint64 ns.');
fraction=idivide(remainder*billion,frequency);whole=seconds*billion;
assert(whole<=intmax('uint64')-fraction,'gpenmpcNative:HostClockOverflow','Nanosecond addition overflow.');
value=whole+fraction; % Exact integer floor.
end
