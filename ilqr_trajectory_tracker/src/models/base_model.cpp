<<<<<<< HEAD
=======
// Copyright (C) 2024 Cihat Kurtuluş Altıparmak
// Copyright (C) 2024 Prof. Dr. Tufan Kumbasar, ITU AI2S Lab
// Copyright (C) 2024 Prof. Dr. Behçet Uğur Töreyin
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

>>>>>>> 4094837 (Format code according to ros standard (#71))
namespace ilqr_trajectory_tracker
{

class Model
{
};

class BaseModel : public Model
{
public:
  BaseModel();
  virtual void apply_system_dynamics() = 0;
  virtual void get_state_matrix() = 0;
  virtual void get_control_matrix() = 0;
};

BaseModel::BaseModel()
{
}

}  // namespace ilqr_trajectory_tracker
