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
% @deftypefn {} {@var{retval} =} plane_meas (@var{input1}, @var{input2})
%
% @seealso{}
% @end deftypefn
%
% Author: matous <matous@SKUMPA-Linux>
% Created: 2020-11-20

function [bases, origin, m] = plane_meas(ground_truth)

  m = 1;
  bases = rand(3, 2);
  bases(:, 1) = bases(:, 1)/norm(bases(:, 1));
  bases(:, 2) = bases(:, 2)/norm(bases(:, 2));
  origin = ground_truth;
  
end
