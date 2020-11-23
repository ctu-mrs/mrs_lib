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
## @deftypefn {} {@var{retval} =} constraint_transform (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

## Author: matous <matous@matous-Myslplacka>
## Created: 2020-11-23

function M = constraint_transform (constrained_states)

  cs = constrained_states > 0;
  n = size(cs, 1);
  c = sum(cs);
  M = zeros(c, n);
  M(:, cs) = eye(c);
  
endfunction
