% Copyright (C) 2020 matous
% 
% This program is free software; you can redistribute it and/or modify it
% under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 3 of the License, or
% (at your option) any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%
% -*- texinfo -*- 
% @deftypefn {} {@var{retval} =} kf (@var{input1}, @var{input2})
%
% @seealso{}
% @end deftypefn
%
% Author: matous <matous@SKUMPA-Linux>
% Created: 2020-11-20

function [xn, Pn] = kf_correct (H, x, P, z, R)

S = H*P*H' + R;
K = P*H'/S;

xn = x + K*(z - H*x);
Pn = P - K*H*P;

end
