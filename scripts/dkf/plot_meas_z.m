% Copyright (C) 2020 matous-viktor
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
% @deftypefn {} {@var{retval} =} plot_meas (@var{input1}, @var{input2})
%
% @seealso{}
% @end deftypefn
%
% Author: matous <matous@SKUMPA-Linux> and viktor
% Created: 2020-11-20

function h=plot_meas (bases, z)

  nbases = null(bases');
  origin = nbases*z;
  switch (size(bases, 2))
    case 1
      pts = [origin - 666*bases, origin + 666*bases];
      h = plot3(pts(1, :), pts(2, :), pts(3, :));
    case 2
      nrm = null(bases');
      [x, y, z] = plane_surf(nrm, origin, 20);
      h = surf(x, y, z,'FaceAlpha',0.5);
  end

end
