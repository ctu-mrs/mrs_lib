## Copyright (C) 2020 matous
## 
## This program is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 3 of the License, or
## (at your option) any later version.
## 
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
## 
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.

## -*- texinfo -*- 
## @deftypefn {} {@var{retval} =} plane_surf (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: matous <matous@SKUMPA-Linux>
## Created: 2020-11-20

function [x, y, z] = plane_surf (normal, center, size)
  normal = normal / norm(normal);

  tangents = null(normal') * size;

  res(1,1,:) = center + tangents * [-1;-1]; 
  res(1,2,:) = center + tangents * [-1;1]; 
  res(2,2,:) = center + tangents * [1;1]; 
  res(2,1,:) = center + tangents * [1;-1];

  x = squeeze(res(:,:,1));
  y = squeeze(res(:,:,2));
  z = squeeze(res(:,:,3));
endfunction
