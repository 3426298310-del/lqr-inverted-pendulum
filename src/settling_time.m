function ts = settling_time(t,y,band)
%SETTLING_TIME First sampled time after the LAST excursion outside +/-band.
% NaN if still outside at the final sample. This is finite-horizon evidence.
validateattributes(t,{'numeric'},{'vector','real','finite','nonempty'});
validateattributes(y,{'numeric'},{'vector','real','finite','numel',numel(t)});
validateattributes(band,{'numeric'},{'scalar','positive','finite'});
t=t(:); y=y(:);
assert(all(diff(t)>0),'Time must be strictly increasing.');
last=find(abs(y)>band,1,'last');
if isempty(last), ts=t(1);
elseif last==numel(t), ts=NaN;
else, ts=t(last+1);
end
end
