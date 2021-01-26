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

function [bases, origin, m, R] = plane_velocity_meas(ground_truth)
  % to measure an intersecting plane and projection of velocity into a line in this plane

  m = 3;
  % bases = rand(3, 2);
  pos_bases = 2*rand(3, 1)-[1;1;1];
  pos_bases(:, 1) = pos_bases(:, 1)/norm(pos_bases(:, 1));
  % pos_bases(:, 2) = pos_bases(:, 2)/norm(pos_bases(:, 2));
  % pos_bases
  % ground_truth
  pos_origin = ground_truth(1:3)+(pos_bases*((rand(1,1)-0.5)*20));

  vel_bases = null(pos_bases');
  % vel_n_bases = ground_truth(1:3) - pos_origin ;
  % vel_n_bases = vel_n_base/norm(vel_n_base);
  % vel_base = null(vel_n_base')
  bases = [pos_bases zeros(3,2); zeros(3,1) vel_bases]
  origin = [pos_origin; zeros(3,1)]

  dx = 0.1;
  dv = 0.5;
  posnull = [null(bases(1:3,1)'), zeros(3,1)];
  velnull = [null(bases(4:6,2:3)'), zeros(3,2)];
  R = [posnull*diag([dx,dx,0])*posnull', zeros(3); zeros(3), velnull*diag([dv,0,0])*velnull'];

  origin = mvnrnd(origin, R)'
end
