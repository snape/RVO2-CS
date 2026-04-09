/*
 * Simulator.cs
 * RVO2 Library C#
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace RVO
{
    /// <summary>Defines the simulation.</summary>
    public class Simulator
    {
        internal IList<Agent> _agents;
        internal IList<Obstacle> _obstacles;
        internal KdTree _kdTree;
        internal float _timeStep;

        private static readonly Simulator _instance = new();

        private Agent _defaultAgent;
        private int _numWorkers;
        private float _globalTime;
        private bool _obstaclesProcessed;

        public static Simulator Instance
        {
            get
            {
                return _instance;
            }
        }

        /// <summary>Adds a new agent with default properties to the simulation.
        /// </summary>
        ///
        /// <returns>The number of the agent, or -1 when the agent defaults have
        /// not been set.</returns>
        ///
        /// <param name="position">The two-dimensional starting position of this
        /// agent.</param>
        public int AddAgent(Vector2 position)
        {
            if (_defaultAgent is null)
            {
                return -1;
            }

            Agent agent = new();
            agent._id = _agents.Count;
            agent._maxNeighbors = _defaultAgent._maxNeighbors;
            agent._maxSpeed = _defaultAgent._maxSpeed;
            agent._neighborDist = _defaultAgent._neighborDist;
            agent._position = position;
            agent._radius = _defaultAgent._radius;
            agent._timeHorizon = _defaultAgent._timeHorizon;
            agent._timeHorizonObst = _defaultAgent._timeHorizonObst;
            agent._velocity = _defaultAgent._velocity;
            _agents.Add(agent);

            return agent._id;
        }

        /// <summary>Adds a new agent with default properties to the simulation.
        /// </summary>
        ///
        /// <returns>The number of the agent, or -1 when the agent defaults have
        /// not been set.</returns>
        ///
        /// <param name="position">The two-dimensional starting position of this
        /// agent.</param>
        [Obsolete("Use AddAgent instead.", false)]
        public int addAgent(Vector2 position)
        {
            return AddAgent(position);
        }

        /// <summary>Adds a new agent to the simulation.</summary>
        ///
        /// <returns>The number of the agent.</returns>
        ///
        /// <param name="position">The two-dimensional starting position of this
        /// agent.</param>
        /// <param name="neighborDist">The maximum distance (center point to
        /// center point) to other agents this agent takes into account in the
        /// navigation. The larger this number, the longer the running time of
        /// the simulation. If the number is too low, the simulation will not be
        /// safe. Must be non-negative.</param>
        /// <param name="maxNeighbors">The maximum number of other agents this
        /// agent takes into account in the navigation. The larger this number,
        /// the longer the running time of the simulation. If the number is too
        /// low, the simulation will not be safe.</param>
        /// <param name="timeHorizon">The minimal amount of time for which this
        /// agent's velocities that are computed by the simulation are safe with
        /// respect to other agents. The larger this number, the sooner this
        /// agent will respond to the presence of other agents, but the less
        /// freedom this agent has in choosing its velocities. Must be positive.
        /// </param>
        /// <param name="timeHorizonObst">The minimal amount of time for which
        /// this agent's velocities that are computed by the simulation are safe
        /// with respect to obstacles. The larger this number, the sooner this
        /// agent will respond to the presence of obstacles, but the less freedom
        /// this agent has in choosing its velocities. Must be positive.</param>
        /// <param name="radius">The radius of this agent. Must be non-negative.
        /// </param>
        /// <param name="maxSpeed">The maximum speed of this agent. Must be
        /// non-negative.</param>
        /// <param name="velocity">The initial two-dimensional linear velocity of
        /// this agent.</param>
        public int AddAgent(Vector2 position, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(maxNeighbors, nameof(maxNeighbors));
            ArgumentOutOfRangeException.ThrowIfNegative(neighborDist, nameof(neighborDist));
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(timeHorizon, nameof(timeHorizon));
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(timeHorizonObst, nameof(timeHorizonObst));
            ArgumentOutOfRangeException.ThrowIfNegative(radius, nameof(radius));
            ArgumentOutOfRangeException.ThrowIfNegative(maxSpeed, nameof(maxSpeed));

            Agent agent = new();
            agent._id = _agents.Count;
            agent._maxNeighbors = maxNeighbors;
            agent._maxSpeed = maxSpeed;
            agent._neighborDist = neighborDist;
            agent._position = position;
            agent._radius = radius;
            agent._timeHorizon = timeHorizon;
            agent._timeHorizonObst = timeHorizonObst;
            agent._velocity = velocity;
            _agents.Add(agent);

            return agent._id;
        }

        /// <summary>Adds a new agent to the simulation.</summary>
        ///
        /// <returns>The number of the agent.</returns>
        ///
        /// <param name="position">The two-dimensional starting position of this
        /// agent.</param>
        /// <param name="neighborDist">The maximum distance (center point to
        /// center point) to other agents this agent takes into account in the
        /// navigation. The larger this number, the longer the running time of
        /// the simulation. If the number is too low, the simulation will not be
        /// safe. Must be non-negative.</param>
        /// <param name="maxNeighbors">The maximum number of other agents this
        /// agent takes into account in the navigation. The larger this number,
        /// the longer the running time of the simulation. If the number is too
        /// low, the simulation will not be safe.</param>
        /// <param name="timeHorizon">The minimal amount of time for which this
        /// agent's velocities that are computed by the simulation are safe with
        /// respect to other agents. The larger this number, the sooner this
        /// agent will respond to the presence of other agents, but the less
        /// freedom this agent has in choosing its velocities. Must be positive.
        /// </param>
        /// <param name="timeHorizonObst">The minimal amount of time for which
        /// this agent's velocities that are computed by the simulation are safe
        /// with respect to obstacles. The larger this number, the sooner this
        /// agent will respond to the presence of obstacles, but the less freedom
        /// this agent has in choosing its velocities. Must be positive.</param>
        /// <param name="radius">The radius of this agent. Must be non-negative.
        /// </param>
        /// <param name="maxSpeed">The maximum speed of this agent. Must be
        /// non-negative.</param>
        /// <param name="velocity">The initial two-dimensional linear velocity of
        /// this agent.</param>
        [Obsolete("Use AddAgent instead.", false)]
        public int addAgent(Vector2 position, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            return AddAgent(position, neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, velocity);
        }

        /// <summary>Adds a new obstacle to the simulation.</summary>
        ///
        /// <returns>The number of the first vertex of the obstacle, or -1 when
        /// the number of vertices is less than two.</returns>
        ///
        /// <param name="vertices">List of the vertices of the polygonal obstacle
        /// in counterclockwise order.</param>
        ///
        /// <remarks>To add a "negative" obstacle, e.g. a bounding polygon around
        /// the environment, the vertices should be listed in clockwise order.
        /// </remarks>
        public int AddObstacle(IList<Vector2> vertices)
        {
            if (vertices.Count < 2)
            {
                return -1;
            }

            for (int i = 0; i < vertices.Count; ++i)
            {
                int next = i == vertices.Count - 1 ? 0 : i + 1;

                if (RVOMath.AbsSq(vertices[next] - vertices[i]) <= RVOMath.RVO_EPSILON * RVOMath.RVO_EPSILON)
                {
                    return -1;
                }
            }

            int obstacleNo = _obstacles.Count;

            for (int i = 0; i < vertices.Count; ++i)
            {
                Obstacle obstacle = new();
                obstacle._point = vertices[i];

                if (i != 0)
                {
                    obstacle._previous = _obstacles[_obstacles.Count - 1];
                    obstacle._previous._next = obstacle;
                }

                if (i == vertices.Count - 1)
                {
                    obstacle._next = _obstacles[obstacleNo];
                    obstacle._next._previous = obstacle;
                }

                obstacle._direction = RVOMath.Normalize(vertices[(i == vertices.Count - 1 ? 0 : i + 1)] - vertices[i]);

                if (vertices.Count == 2)
                {
                    obstacle._convex = true;
                }
                else
                {
                    obstacle._convex = (RVOMath.LeftOf(vertices[(i == 0 ? vertices.Count - 1 : i - 1)], vertices[i], vertices[(i == vertices.Count - 1 ? 0 : i + 1)]) >= 0.0f);
                }

                obstacle._id = _obstacles.Count;
                _obstacles.Add(obstacle);
            }

            return obstacleNo;
        }

        /// <summary>Adds a new obstacle to the simulation.</summary>
        ///
        /// <returns>The number of the first vertex of the obstacle, or -1 when
        /// the number of vertices is less than two.</returns>
        ///
        /// <param name="vertices">List of the vertices of the polygonal obstacle
        /// in counterclockwise order.</param>
        ///
        /// <remarks>To add a "negative" obstacle, e.g. a bounding polygon around
        /// the environment, the vertices should be listed in clockwise order.
        /// </remarks>
        [Obsolete("Use AddObstacle instead.", false)]
        public int addObstacle(IList<Vector2> vertices)
        {
            return AddObstacle(vertices);
        }

        /// <summary>Clears the simulation.</summary>
        public void Clear()
        {
            _agents = new List<Agent>();
            _defaultAgent = null;
            _kdTree = new KdTree();
            _obstacles = new List<Obstacle>();
            _globalTime = 0.0f;
            _obstaclesProcessed = false;
            _timeStep = 0.1f;

            NumWorkers = 0;
        }

        /// <summary>Performs a simulation step and updates the two-dimensional
        /// position and two-dimensional velocity of each agent.</summary>
        ///
        /// <returns>The global time after the simulation step.</returns>
        public float DoStep()
        {
            ParallelOptions options = new() { MaxDegreeOfParallelism = _numWorkers };

            _kdTree.BuildAgentTree();

            Parallel.For(0, _agents.Count, options, i =>
            {
                _agents[i].ComputeNeighbors();
                _agents[i].ComputeNewVelocity();
            });

            Parallel.For(0, _agents.Count, options, i => _agents[i].Update());

            _globalTime += _timeStep;

            return _globalTime;
        }

        /// <summary>Performs a simulation step and updates the two-dimensional
        /// position and two-dimensional velocity of each agent.</summary>
        ///
        /// <returns>The global time after the simulation step.</returns>
        [Obsolete("Use DoStep instead.", false)]
        public float doStep()
        {
            return DoStep();
        }

        /// <summary>Returns the specified agent neighbor of the specified agent.
        /// </summary>
        ///
        /// <returns>The number of the neighboring agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose agent neighbor is
        /// to be retrieved.</param>
        /// <param name="neighborNo">The number of the agent neighbor to be
        /// retrieved.</param>
        public int GetAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegative(neighborNo, nameof(neighborNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(neighborNo, _agents[agentNo]._agentNeighbors.Count, nameof(neighborNo));

            return _agents[agentNo]._agentNeighbors[neighborNo].Value._id;
        }

        /// <summary>Returns the specified agent neighbor of the specified agent.
        /// </summary>
        ///
        /// <returns>The number of the neighboring agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose agent neighbor is
        /// to be retrieved.</param>
        /// <param name="neighborNo">The number of the agent neighbor to be
        /// retrieved.</param>
        [Obsolete("Use GetAgentAgentNeighbor instead.", false)]
        public int getAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            return GetAgentAgentNeighbor(agentNo, neighborNo);
        }

        /// <summary>Returns the maximum neighbor count of a specified agent.
        /// </summary>
        ///
        /// <returns>The present maximum neighbor count of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// count is to be retrieved.</param>
        public int GetAgentMaxNeighbors(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._maxNeighbors;
        }

        /// <summary>Returns the maximum neighbor count of a specified agent.
        /// </summary>
        ///
        /// <returns>The present maximum neighbor count of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// count is to be retrieved.</param>
        [Obsolete("Use GetAgentMaxNeighbors instead.", false)]
        public int getAgentMaxNeighbors(int agentNo)
        {
            return GetAgentMaxNeighbors(agentNo);
        }

        /// <summary>Returns the maximum speed of a specified agent.</summary>
        ///
        /// <returns>The present maximum speed of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose maximum speed is
        /// to be retrieved.</param>
        public float GetAgentMaxSpeed(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._maxSpeed;
        }

        /// <summary>Returns the maximum speed of a specified agent.</summary>
        ///
        /// <returns>The present maximum speed of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose maximum speed is
        /// to be retrieved.</param>
        [Obsolete("Use GetAgentMaxSpeed instead.", false)]
        public float getAgentMaxSpeed(int agentNo)
        {
            return GetAgentMaxSpeed(agentNo);
        }

        /// <summary>Returns the maximum neighbor distance of a specified agent.
        /// </summary>
        ///
        /// <returns>The present maximum neighbor distance of the agent.
        /// </returns>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// distance is to be retrieved.</param>
        public float GetAgentNeighborDist(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._neighborDist;
        }

        /// <summary>Returns the maximum neighbor distance of a specified agent.
        /// </summary>
        ///
        /// <returns>The present maximum neighbor distance of the agent.
        /// </returns>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// distance is to be retrieved.</param>
        [Obsolete("Use GetAgentNeighborDist instead.", false)]
        public float getAgentNeighborDist(int agentNo)
        {
            return GetAgentNeighborDist(agentNo);
        }

        /// <summary>Returns the count of agent neighbors taken into account to
        /// compute the current velocity for the specified agent.</summary>
        ///
        /// <returns>The count of agent neighbors taken into account to compute
        /// the current velocity for the specified agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose count of agent
        /// neighbors is to be retrieved.</param>
        public int GetAgentNumAgentNeighbors(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._agentNeighbors.Count;
        }

        /// <summary>Returns the count of agent neighbors taken into account to
        /// compute the current velocity for the specified agent.</summary>
        ///
        /// <returns>The count of agent neighbors taken into account to compute
        /// the current velocity for the specified agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose count of agent
        /// neighbors is to be retrieved.</param>
        [Obsolete("Use GetAgentNumAgentNeighbors instead.", false)]
        public int getAgentNumAgentNeighbors(int agentNo)
        {
            return GetAgentNumAgentNeighbors(agentNo);
        }

        /// <summary>Returns the count of obstacle neighbors taken into account
        /// to compute the current velocity for the specified agent.</summary>
        ///
        /// <returns>The count of obstacle neighbors taken into account to
        /// compute the current velocity for the specified agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose count of obstacle
        /// neighbors is to be retrieved.</param>
        public int GetAgentNumObstacleNeighbors(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._obstacleNeighbors.Count;
        }

        /// <summary>Returns the count of obstacle neighbors taken into account
        /// to compute the current velocity for the specified agent.</summary>
        ///
        /// <returns>The count of obstacle neighbors taken into account to
        /// compute the current velocity for the specified agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose count of obstacle
        /// neighbors is to be retrieved.</param>
        [Obsolete("Use GetAgentNumObstacleNeighbors instead.", false)]
        public int getAgentNumObstacleNeighbors(int agentNo)
        {
            return GetAgentNumObstacleNeighbors(agentNo);
        }

        /// <summary>Returns the specified obstacle neighbor of the specified
        /// agent.</summary>
        ///
        /// <returns>The number of the first vertex of the neighboring obstacle
        /// edge.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose obstacle neighbor
        /// is to be retrieved.</param>
        /// <param name="neighborNo">The number of the obstacle neighbor to be
        /// retrieved.</param>
        public int GetAgentObstacleNeighbor(int agentNo, int neighborNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegative(neighborNo, nameof(neighborNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(neighborNo, _agents[agentNo]._obstacleNeighbors.Count, nameof(neighborNo));

            return _agents[agentNo]._obstacleNeighbors[neighborNo].Value._id;
        }

        /// <summary>Returns the specified obstacle neighbor of the specified
        /// agent.</summary>
        ///
        /// <returns>The number of the first vertex of the neighboring obstacle
        /// edge.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose obstacle neighbor
        /// is to be retrieved.</param>
        /// <param name="neighborNo">The number of the obstacle neighbor to be
        /// retrieved.</param>
        [Obsolete("Use GetAgentObstacleNeighbor instead.", false)]
        public int getAgentObstacleNeighbor(int agentNo, int neighborNo)
        {
            return GetAgentObstacleNeighbor(agentNo, neighborNo);
        }

        /// <summary>Returns the ORCA constraints of the specified agent.
        /// </summary>
        ///
        /// <returns>A list of lines representing the ORCA constraints.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose ORCA constraints
        /// are to be retrieved.</param>
        ///
        /// <remarks>The halfplane to the left of each line is the region of
        /// permissible velocities with respect to that ORCA constraint.
        /// </remarks>
        public IList<Line> GetAgentOrcaLines(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return new List<Line>(_agents[agentNo]._orcaLines);
        }

        /// <summary>Returns the ORCA constraints of the specified agent.
        /// </summary>
        ///
        /// <returns>A list of lines representing the ORCA constraints.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose ORCA constraints
        /// are to be retrieved.</param>
        ///
        /// <remarks>The halfplane to the left of each line is the region of
        /// permissible velocities with respect to that ORCA constraint.
        /// </remarks>
        [Obsolete("Use GetAgentOrcaLines instead.", false)]
        public IList<Line> getAgentOrcaLines(int agentNo)
        {
            return GetAgentOrcaLines(agentNo);
        }

        /// <summary>Returns the two-dimensional position of a specified agent.
        /// </summary>
        ///
        /// <returns>The present two-dimensional position of the (center of the)
        /// agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// position is to be retrieved.</param>
        public Vector2 GetAgentPosition(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._position;
        }

        /// <summary>Returns the two-dimensional position of a specified agent.
        /// </summary>
        ///
        /// <returns>The present two-dimensional position of the (center of the)
        /// agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// position is to be retrieved.</param>
        [Obsolete("Use GetAgentPosition instead.", false)]
        public Vector2 getAgentPosition(int agentNo)
        {
            return GetAgentPosition(agentNo);
        }

        /// <summary>Returns the two-dimensional preferred velocity of a
        /// specified agent.</summary>
        ///
        /// <returns>The present two-dimensional preferred velocity of the agent.
        /// </returns>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// preferred velocity is to be retrieved.</param>
        public Vector2 GetAgentPrefVelocity(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._prefVelocity;
        }

        /// <summary>Returns the two-dimensional preferred velocity of a
        /// specified agent.</summary>
        ///
        /// <returns>The present two-dimensional preferred velocity of the agent.
        /// </returns>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// preferred velocity is to be retrieved.</param>
        [Obsolete("Use GetAgentPrefVelocity instead.", false)]
        public Vector2 getAgentPrefVelocity(int agentNo)
        {
            return GetAgentPrefVelocity(agentNo);
        }

        /// <summary>Returns the radius of a specified agent.</summary>
        ///
        /// <returns>The present radius of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose radius is to be
        /// retrieved.</param>
        public float GetAgentRadius(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._radius;
        }

        /// <summary>Returns the radius of a specified agent.</summary>
        ///
        /// <returns>The present radius of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose radius is to be
        /// retrieved.</param>
        [Obsolete("Use GetAgentRadius instead.", false)]
        public float getAgentRadius(int agentNo)
        {
            return GetAgentRadius(agentNo);
        }

        /// <summary>Returns the time horizon of a specified agent.</summary>
        ///
        /// <returns>The present time horizon of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon is
        /// to be retrieved.</param>
        public float GetAgentTimeHorizon(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._timeHorizon;
        }

        /// <summary>Returns the time horizon of a specified agent.</summary>
        ///
        /// <returns>The present time horizon of the agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon is
        /// to be retrieved.</param>
        [Obsolete("Use GetAgentTimeHorizon instead.", false)]
        public float getAgentTimeHorizon(int agentNo)
        {
            return GetAgentTimeHorizon(agentNo);
        }

        /// <summary>Returns the time horizon with respect to obstacles of a
        /// specified agent.</summary>
        ///
        /// <returns>The present time horizon with respect to obstacles of the
        /// agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon with
        /// respect to obstacles is to be retrieved.</param>
        public float GetAgentTimeHorizonObst(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._timeHorizonObst;
        }

        /// <summary>Returns the time horizon with respect to obstacles of a
        /// specified agent.</summary>
        ///
        /// <returns>The present time horizon with respect to obstacles of the
        /// agent.</returns>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon with
        /// respect to obstacles is to be retrieved.</param>
        [Obsolete("Use GetAgentTimeHorizonObst instead.", false)]
        public float getAgentTimeHorizonObst(int agentNo)
        {
            return GetAgentTimeHorizonObst(agentNo);
        }

        /// <summary>Returns the two-dimensional linear velocity of a specified
        /// agent.</summary>
        ///
        /// <returns>The present two-dimensional linear velocity of the agent.
        /// </returns>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// linear velocity is to be retrieved.</param>
        public Vector2 GetAgentVelocity(int agentNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            return _agents[agentNo]._velocity;
        }

        /// <summary>Returns the two-dimensional linear velocity of a specified
        /// agent.</summary>
        ///
        /// <returns>The present two-dimensional linear velocity of the agent.
        /// </returns>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// linear velocity is to be retrieved.</param>
        [Obsolete("Use GetAgentVelocity instead.", false)]
        public Vector2 getAgentVelocity(int agentNo)
        {
            return GetAgentVelocity(agentNo);
        }

        /// <summary>Gets the global time of the simulation.</summary>
        ///
        /// <value>The present global time of the simulation (zero initially).
        /// </value>
        public float GlobalTime => _globalTime;

        /// <summary>Returns the global time of the simulation.</summary>
        ///
        /// <returns>The present global time of the simulation (zero initially).
        /// </returns>
        [Obsolete("Use GlobalTime instead.", false)]
        public float getGlobalTime()
        {
            return GlobalTime;
        }

        /// <summary>Gets the count of agents in the simulation.</summary>
        ///
        /// <value>The count of agents in the simulation.</value>
        public int NumAgents => _agents.Count;

        /// <summary>Returns the count of agents in the simulation.</summary>
        ///
        /// <returns>The count of agents in the simulation.</returns>
        [Obsolete("Use NumAgents instead.", false)]
        public int getNumAgents()
        {
            return NumAgents;
        }

        /// <summary>Gets the count of obstacle vertices in the simulation.
        /// </summary>
        ///
        /// <value>The count of obstacle vertices in the simulation.</value>
        public int NumObstacleVertices => _obstacles.Count;

        /// <summary>Returns the count of obstacle vertices in the simulation.
        /// </summary>
        ///
        /// <returns>The count of obstacle vertices in the simulation.</returns>
        [Obsolete("Use NumObstacleVertices instead.", false)]
        public int getNumObstacleVertices()
        {
            return NumObstacleVertices;
        }

        /// <summary>Gets or sets the maximum degree of parallelism for the
        /// simulation step.</summary>
        ///
        /// <value>The maximum number of concurrent tasks used during DoStep. A
        /// value of zero or less imposes no limit.</value>
        public int NumWorkers
        {
            get => _numWorkers;
            set => _numWorkers = value <= 0 ? -1 : value;
        }

        /// <summary>Returns the two-dimensional position of a specified obstacle
        /// vertex.</summary>
        ///
        /// <returns>The two-dimensional position of the specified obstacle
        /// vertex.</returns>
        ///
        /// <param name="vertexNo">The number of the obstacle vertex to be
        /// retrieved.</param>
        public Vector2 GetObstacleVertex(int vertexNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(vertexNo, nameof(vertexNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(vertexNo, _obstacles.Count, nameof(vertexNo));

            return _obstacles[vertexNo]._point;
        }

        /// <summary>Returns the two-dimensional position of a specified obstacle
        /// vertex.</summary>
        ///
        /// <returns>The two-dimensional position of the specified obstacle
        /// vertex.</returns>
        ///
        /// <param name="vertexNo">The number of the obstacle vertex to be
        /// retrieved.</param>
        [Obsolete("Use GetObstacleVertex instead.", false)]
        public Vector2 getObstacleVertex(int vertexNo)
        {
            return GetObstacleVertex(vertexNo);
        }

        /// <summary>Returns the number of the obstacle vertex succeeding the
        /// specified obstacle vertex in its polygon.</summary>
        ///
        /// <returns>The number of the obstacle vertex succeeding the specified
        /// obstacle vertex in its polygon.</returns>
        ///
        /// <param name="vertexNo">The number of the obstacle vertex whose
        /// successor is to be retrieved.</param>
        public int GetNextObstacleVertexNo(int vertexNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(vertexNo, nameof(vertexNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(vertexNo, _obstacles.Count, nameof(vertexNo));

            return _obstacles[vertexNo]._next._id;
        }

        /// <summary>Returns the number of the obstacle vertex succeeding the
        /// specified obstacle vertex in its polygon.</summary>
        ///
        /// <returns>The number of the obstacle vertex succeeding the specified
        /// obstacle vertex in its polygon.</returns>
        ///
        /// <param name="vertexNo">The number of the obstacle vertex whose
        /// successor is to be retrieved.</param>
        [Obsolete("Use GetNextObstacleVertexNo instead.", false)]
        public int getNextObstacleVertexNo(int vertexNo)
        {
            return GetNextObstacleVertexNo(vertexNo);
        }

        /// <summary>Returns the number of the obstacle vertex preceding the
        /// specified obstacle vertex in its polygon.</summary>
        ///
        /// <returns>The number of the obstacle vertex preceding the specified
        /// obstacle vertex in its polygon.</returns>
        ///
        /// <param name="vertexNo">The number of the obstacle vertex whose
        /// predecessor is to be retrieved.</param>
        public int GetPrevObstacleVertexNo(int vertexNo)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(vertexNo, nameof(vertexNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(vertexNo, _obstacles.Count, nameof(vertexNo));

            return _obstacles[vertexNo]._previous._id;
        }

        /// <summary>Returns the number of the obstacle vertex preceding the
        /// specified obstacle vertex in its polygon.</summary>
        ///
        /// <returns>The number of the obstacle vertex preceding the specified
        /// obstacle vertex in its polygon.</returns>
        ///
        /// <param name="vertexNo">The number of the obstacle vertex whose
        /// predecessor is to be retrieved.</param>
        [Obsolete("Use GetPrevObstacleVertexNo instead.", false)]
        public int getPrevObstacleVertexNo(int vertexNo)
        {
            return GetPrevObstacleVertexNo(vertexNo);
        }

        /// <summary>Gets or sets the time step of the simulation.</summary>
        ///
        /// <value>The time step of the simulation. Must be positive.</value>
        public float TimeStep
        {
            get => _timeStep;
            set
            {
                ArgumentOutOfRangeException.ThrowIfNegativeOrZero(value, nameof(value));

                _timeStep = value;
            }
        }

        /// <summary>Returns the time step of the simulation.</summary>
        ///
        /// <returns>The present time step of the simulation.</returns>
        [Obsolete("Use TimeStep instead.", false)]
        public float getTimeStep()
        {
            return TimeStep;
        }

        /// <summary>Processes the obstacles that have been added so that they
        /// are accounted for in the simulation.</summary>
        ///
        /// <remarks>Obstacles added to the simulation after this function has
        /// been called are not accounted for in the simulation.</remarks>
        public void ProcessObstacles()
        {
            if (_obstaclesProcessed)
            {
                throw new InvalidOperationException("ProcessObstacles has already been called. Call Clear() to reset the simulation before processing obstacles again.");
            }

            _kdTree.BuildObstacleTree();
            _obstaclesProcessed = true;
        }

        /// <summary>Processes the obstacles that have been added so that they
        /// are accounted for in the simulation.</summary>
        ///
        /// <remarks>Obstacles added to the simulation after this function has
        /// been called are not accounted for in the simulation.</remarks>
        [Obsolete("Use ProcessObstacles instead.", false)]
        public void processObstacles()
        {
            ProcessObstacles();
        }

        /// <summary>Performs a visibility query between the two specified points
        /// with respect to the obstacles.</summary>
        ///
        /// <returns>A boolean specifying whether the two points are mutually
        /// visible. Returns true when the obstacles have not been processed.
        /// </returns>
        ///
        /// <param name="point1">The first point of the query.</param>
        /// <param name="point2">The second point of the query.</param>
        /// <param name="radius">The minimal distance between the line connecting
        /// the two points and the obstacles in order for the points to be
        /// mutually visible (optional). Must be non-negative.</param>
        public bool QueryVisibility(Vector2 point1, Vector2 point2, float radius)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(radius, nameof(radius));

            if (RVOMath.AbsSq(point2 - point1) <= RVOMath.RVO_EPSILON * RVOMath.RVO_EPSILON)
            {
                return true;
            }

            return _kdTree.QueryVisibility(point1, point2, radius);
        }

        /// <summary>Performs a visibility query between the two specified points
        /// with respect to the obstacles.</summary>
        ///
        /// <returns>A boolean specifying whether the two points are mutually
        /// visible. Returns true when the obstacles have not been processed.
        /// </returns>
        ///
        /// <param name="point1">The first point of the query.</param>
        /// <param name="point2">The second point of the query.</param>
        /// <param name="radius">The minimal distance between the line connecting
        /// the two points and the obstacles in order for the points to be
        /// mutually visible (optional). Must be non-negative.</param>
        [Obsolete("Use QueryVisibility instead.", false)]
        public bool queryVisibility(Vector2 point1, Vector2 point2, float radius)
        {
            return QueryVisibility(point1, point2, radius);
        }

        /// <summary>Sets the default properties for any new agent that is added.
        /// </summary>
        ///
        /// <param name="neighborDist">The default maximum distance (center point
        /// to center point) to other agents a new agent takes into account in
        /// the navigation. The larger this number, the longer the running time of
        /// the simulation. If the number is too low, the simulation will not be
        /// safe. Must be non-negative.</param>
        /// <param name="maxNeighbors">The default maximum number of other agents
        /// a new agent takes into account in the navigation. The larger this
        /// number, the longer the running time of the simulation. If the number
        /// is too low, the simulation will not be safe.</param>
        /// <param name="timeHorizon">The default minimal amount of time for
        /// which a new agent's velocities that are computed by the simulation
        /// are safe with respect to other agents. The larger this number, the
        /// sooner an agent will respond to the presence of other agents, but the
        /// less freedom the agent has in choosing its velocities. Must be
        /// positive.</param>
        /// <param name="timeHorizonObst">The default minimal amount of time for
        /// which a new agent's velocities that are computed by the simulation
        /// are safe with respect to obstacles. The larger this number, the
        /// sooner an agent will respond to the presence of obstacles, but the
        /// less freedom the agent has in choosing its velocities. Must be
        /// positive.</param>
        /// <param name="radius">The default radius of a new agent. Must be
        /// non-negative.</param>
        /// <param name="maxSpeed">The default maximum speed of a new agent. Must
        /// be non-negative.</param>
        /// <param name="velocity">The default initial two-dimensional linear
        /// velocity of a new agent.</param>
        public void SetAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(maxNeighbors, nameof(maxNeighbors));
            ArgumentOutOfRangeException.ThrowIfNegative(neighborDist, nameof(neighborDist));
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(timeHorizon, nameof(timeHorizon));
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(timeHorizonObst, nameof(timeHorizonObst));
            ArgumentOutOfRangeException.ThrowIfNegative(radius, nameof(radius));
            ArgumentOutOfRangeException.ThrowIfNegative(maxSpeed, nameof(maxSpeed));

            if (_defaultAgent is null)
            {
                _defaultAgent = new Agent();
            }

            _defaultAgent._maxNeighbors = maxNeighbors;
            _defaultAgent._maxSpeed = maxSpeed;
            _defaultAgent._neighborDist = neighborDist;
            _defaultAgent._radius = radius;
            _defaultAgent._timeHorizon = timeHorizon;
            _defaultAgent._timeHorizonObst = timeHorizonObst;
            _defaultAgent._velocity = velocity;
        }

        /// <summary>Sets the default properties for any new agent that is added.
        /// </summary>
        ///
        /// <param name="neighborDist">The default maximum distance (center point
        /// to center point) to other agents a new agent takes into account in
        /// the navigation. The larger this number, the longer the running time of
        /// the simulation. If the number is too low, the simulation will not be
        /// safe. Must be non-negative.</param>
        /// <param name="maxNeighbors">The default maximum number of other agents
        /// a new agent takes into account in the navigation. The larger this
        /// number, the longer the running time of the simulation. If the number
        /// is too low, the simulation will not be safe.</param>
        /// <param name="timeHorizon">The default minimal amount of time for
        /// which a new agent's velocities that are computed by the simulation
        /// are safe with respect to other agents. The larger this number, the
        /// sooner an agent will respond to the presence of other agents, but the
        /// less freedom the agent has in choosing its velocities. Must be
        /// positive.</param>
        /// <param name="timeHorizonObst">The default minimal amount of time for
        /// which a new agent's velocities that are computed by the simulation
        /// are safe with respect to obstacles. The larger this number, the
        /// sooner an agent will respond to the presence of obstacles, but the
        /// less freedom the agent has in choosing its velocities. Must be
        /// positive.</param>
        /// <param name="radius">The default radius of a new agent. Must be
        /// non-negative.</param>
        /// <param name="maxSpeed">The default maximum speed of a new agent. Must
        /// be non-negative.</param>
        /// <param name="velocity">The default initial two-dimensional linear
        /// velocity of a new agent.</param>
        [Obsolete("Use SetAgentDefaults instead.", false)]
        public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            SetAgentDefaults(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst, radius, maxSpeed, velocity);
        }

        /// <summary>Sets the maximum neighbor count of a specified agent.
        /// </summary>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// count is to be modified.</param>
        /// <param name="maxNeighbors">The replacement maximum neighbor count.
        /// Must be non-negative.</param>
        public void SetAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegative(maxNeighbors, nameof(maxNeighbors));

            _agents[agentNo]._maxNeighbors = maxNeighbors;
        }

        /// <summary>Sets the maximum neighbor count of a specified agent.
        /// </summary>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// count is to be modified.</param>
        /// <param name="maxNeighbors">The replacement maximum neighbor count.
        /// </param>
        [Obsolete("Use SetAgentMaxNeighbors instead.", false)]
        public void setAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            SetAgentMaxNeighbors(agentNo, maxNeighbors);
        }

        /// <summary>Sets the maximum speed of a specified agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose maximum speed is
        /// to be modified.</param>
        /// <param name="maxSpeed">The replacement maximum speed. Must be
        /// non-negative.</param>
        public void SetAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegative(maxSpeed, nameof(maxSpeed));

            _agents[agentNo]._maxSpeed = maxSpeed;
        }

        /// <summary>Sets the maximum speed of a specified agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose maximum speed is
        /// to be modified.</param>
        /// <param name="maxSpeed">The replacement maximum speed. Must be
        /// non-negative.</param>
        [Obsolete("Use SetAgentMaxSpeed instead.", false)]
        public void setAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            SetAgentMaxSpeed(agentNo, maxSpeed);
        }

        /// <summary>Sets the maximum neighbor distance of a specified agent.
        /// </summary>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// distance is to be modified.</param>
        /// <param name="neighborDist">The replacement maximum neighbor distance.
        /// Must be non-negative.</param>
        public void SetAgentNeighborDist(int agentNo, float neighborDist)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegative(neighborDist, nameof(neighborDist));

            _agents[agentNo]._neighborDist = neighborDist;
        }

        /// <summary>Sets the maximum neighbor distance of a specified agent.
        /// </summary>
        ///
        /// <param name="agentNo">The number of the agent whose maximum neighbor
        /// distance is to be modified.</param>
        /// <param name="neighborDist">The replacement maximum neighbor distance.
        /// Must be non-negative.</param>
        [Obsolete("Use SetAgentNeighborDist instead.", false)]
        public void setAgentNeighborDist(int agentNo, float neighborDist)
        {
            SetAgentNeighborDist(agentNo, neighborDist);
        }

        /// <summary>Sets the two-dimensional position of a specified agent.
        /// </summary>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// position is to be modified.</param>
        /// <param name="position">The replacement of the two-dimensional
        /// position.</param>
        public void SetAgentPosition(int agentNo, Vector2 position)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            _agents[agentNo]._position = position;
        }

        /// <summary>Sets the two-dimensional position of a specified agent.
        /// </summary>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// position is to be modified.</param>
        /// <param name="position">The replacement of the two-dimensional
        /// position.</param>
        [Obsolete("Use SetAgentPosition instead.", false)]
        public void setAgentPosition(int agentNo, Vector2 position)
        {
            SetAgentPosition(agentNo, position);
        }

        /// <summary>Sets the two-dimensional preferred velocity of a specified
        /// agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// preferred velocity is to be modified.</param>
        /// <param name="prefVelocity">The replacement of the two-dimensional
        /// preferred velocity.</param>
        public void SetAgentPrefVelocity(int agentNo, Vector2 prefVelocity)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            _agents[agentNo]._prefVelocity = prefVelocity;
        }

        /// <summary>Sets the two-dimensional preferred velocity of a specified
        /// agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// preferred velocity is to be modified.</param>
        /// <param name="prefVelocity">The replacement of the two-dimensional
        /// preferred velocity.</param>
        [Obsolete("Use SetAgentPrefVelocity instead.", false)]
        public void setAgentPrefVelocity(int agentNo, Vector2 prefVelocity)
        {
            SetAgentPrefVelocity(agentNo, prefVelocity);
        }

        /// <summary>Sets the radius of a specified agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose radius is to be
        /// modified.</param>
        /// <param name="radius">The replacement radius. Must be non-negative.
        /// </param>
        public void SetAgentRadius(int agentNo, float radius)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegative(radius, nameof(radius));

            _agents[agentNo]._radius = radius;
        }

        /// <summary>Sets the radius of a specified agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose radius is to be
        /// modified.</param>
        /// <param name="radius">The replacement radius. Must be non-negative.
        /// </param>
        [Obsolete("Use SetAgentRadius instead.", false)]
        public void setAgentRadius(int agentNo, float radius)
        {
            SetAgentRadius(agentNo, radius);
        }

        /// <summary>Sets the time horizon of a specified agent with respect to
        /// other agents.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon is
        /// to be modified.</param>
        /// <param name="timeHorizon">The replacement time horizon with respect
        /// to other agents. Must be positive.</param>
        public void SetAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(timeHorizon, nameof(timeHorizon));

            _agents[agentNo]._timeHorizon = timeHorizon;
        }

        /// <summary>Sets the time horizon of a specified agent with respect to
        /// other agents.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon is
        /// to be modified.</param>
        /// <param name="timeHorizon">The replacement time horizon with respect
        /// to other agents. Must be positive.</param>
        [Obsolete("Use SetAgentTimeHorizon instead.", false)]
        public void setAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            SetAgentTimeHorizon(agentNo, timeHorizon);
        }

        /// <summary>Sets the time horizon of a specified agent with respect to
        /// obstacles.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon with
        /// respect to obstacles is to be modified.</param>
        /// <param name="timeHorizonObst">The replacement time horizon with
        /// respect to obstacles. Must be positive.</param>
        public void SetAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfNegativeOrZero(timeHorizonObst, nameof(timeHorizonObst));

            _agents[agentNo]._timeHorizonObst = timeHorizonObst;
        }

        /// <summary>Sets the time horizon of a specified agent with respect to
        /// obstacles.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose time horizon with
        /// respect to obstacles is to be modified.</param>
        /// <param name="timeHorizonObst">The replacement time horizon with
        /// respect to obstacles. Must be positive.</param>
        [Obsolete("Use SetAgentTimeHorizonObst instead.", false)]
        public void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            SetAgentTimeHorizonObst(agentNo, timeHorizonObst);
        }

        /// <summary>Sets the two-dimensional linear velocity of a specified
        /// agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// linear velocity is to be modified.</param>
        /// <param name="velocity">The replacement two-dimensional linear
        /// velocity.</param>
        public void SetAgentVelocity(int agentNo, Vector2 velocity)
        {
            ArgumentOutOfRangeException.ThrowIfNegative(agentNo, nameof(agentNo));
            ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(agentNo, _agents.Count, nameof(agentNo));

            _agents[agentNo]._velocity = velocity;
        }

        /// <summary>Sets the two-dimensional linear velocity of a specified
        /// agent.</summary>
        ///
        /// <param name="agentNo">The number of the agent whose two-dimensional
        /// linear velocity is to be modified.</param>
        /// <param name="velocity">The replacement two-dimensional linear
        /// velocity.</param>
        [Obsolete("Use SetAgentVelocity instead.", false)]
        public void setAgentVelocity(int agentNo, Vector2 velocity)
        {
            SetAgentVelocity(agentNo, velocity);
        }

        /// <summary>Sets the global time of the simulation.</summary>
        ///
        /// <param name="globalTime">The global time of the simulation.</param>
        public void SetGlobalTime(float globalTime)
        {
            _globalTime = globalTime;
        }

        /// <summary>Sets the global time of the simulation.</summary>
        ///
        /// <param name="globalTime">The global time of the simulation.</param>
        [Obsolete("Use SetGlobalTime instead.", false)]
        public void setGlobalTime(float globalTime)
        {
            SetGlobalTime(globalTime);
        }

        /// <summary>Sets the time step of the simulation.</summary>
        ///
        /// <param name="timeStep">The time step of the simulation. Must be
        /// positive.</param>
        [Obsolete("Use TimeStep instead.", false)]
        public void setTimeStep(float timeStep)
        {
            TimeStep = timeStep;
        }

        /// <summary>Constructs and initializes a simulation.</summary>
        private Simulator()
        {
            Clear();
        }
    }
}
