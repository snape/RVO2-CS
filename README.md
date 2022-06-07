<!--
README.md
RVO2 Library C#

SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
SPDX-License-Identifier: CC-BY-SA-4.0

Creative Commons Attribution-ShareAlike 4.0 International Public License

You are free to:

* Share -- copy and redistribute the material in any medium or format

* ShareAlike -- If you remix, transform, or build upon the material, you must
  distribute your contributions under the same license as the original

* Adapt -- remix, transform, and build upon the material for any purpose, even
  commercially.

The licensor cannot revoke these freedoms as long as you follow the license
terms.

Under the following terms:

* Attribution -- You must give appropriate credit, provide a link to the
  license, and indicate if changes were made. You may do so in any reasonable
  manner, but not in any way that suggests the licensor endorses you or your
  use.

* No additional restrictions -- You may not apply legal terms or technological
  measures that legally restrict others from doing anything the license
  permits.

Notices:

* You do not have to comply with the license for elements of the material in
  the public domain or where your use is permitted by an applicable exception
  or limitation.

* No warranties are given. The license may not give you all of the permissions
  necessary for your intended use. For example, other rights such as publicity,
  privacy, or moral rights may limit how you use the material.

Please send all bug reports to <geom@cs.unc.edu>.

The authors may be contacted via:

Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
Dept. of Computer Science
201 S. Columbia St.
Frederick P. Brooks, Jr. Computer Science Bldg.
Chapel Hill, N.C. 27599-3175
United States of America

<https://gamma.cs.unc.edu/RVO2/>
-->

Optimal Reciprocal Collision Avoidance for C#
=============================================

<https://gamma.cs.unc.edu/RVO2/>

We present a formal approach to reciprocal collision avoidance, where multiple
independent mobile robots or agents need to avoid collisions with each other
without communication among agents while moving in a common workspace. Our
formulation, optimal reciprocal collision avoidance (ORCA), provides sufficient
conditions for collision-free motion by letting each agent take half of the
responsibility of avoiding pairwise collisions. Selecting the optimal action for
each agent is reduced to solving a low-dimensional linear program, and we prove
that the resulting motions are smooth. We test our optimal reciprocal collision
avoidance approach on several dense and complex simulation scenarios workspaces
involving thousands of agents, and compute collision-free actions for all of
them in only a few milliseconds.

RVO2 Library C# is an open-source C# .NET 5 implementation of our algorithm in
two dimensions. It has a simple API for third-party applications. The user
specifies static obstacles, agents, and the preferred velocities of the agents.
The simulation is performed step-by-step via a simple call to the library. The
simulation is fully accessible and manipulable during runtime.

![Build Status](https://github.com/snape/RVO2-CS/workflows/ci/badge.svg?branch=main)

<!-- REUSE-IgnoreStart -->
SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

&nbsp;&nbsp;<https://www.apache.org/licenses/LICENSE-2.0>

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

Please send all bug reports to [geom@cs.unc.edu](mailto:geom@cs.unc.edu).

The authors may be contacted via:

Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha  
Dept. of Computer Science  
201 S. Columbia St.  
Frederick P. Brooks, Jr. Computer Science Bldg.  
Chapel Hill, N.C. 27599-3175  
United States of America
<!-- REUSE-IgnoreEnd -->
