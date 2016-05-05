/*
 * Simulator.cs
 * RVO2 Library C#
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
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
using System.Threading;

namespace RVO
{
    /**
     * <summary>Defines the simulation.</summary>
     */
    public class Simulator
    {
        /**
         * <summary>Defines a worker.</summary>
         */
        private class Worker
        {
            private ManualResetEvent doneEvent_;
            private int end_;
            private int start_;

            /**
             * <summary>Constructs and initializes a worker.</summary>
             *
             * <param name="start">Start.</param>
             * <param name="end">End.</param>
             * <param name="doneEvent">Done event.</param>
             */
            internal Worker(int start, int end, ManualResetEvent doneEvent)
            {
                start_ = start;
                end_ = end;
                doneEvent_ = doneEvent;
            }

            /**
             * <summary>Performs a simulation step.</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void step(object obj)
            {
                for (int agentNo = start_; agentNo < end_; ++agentNo)
                {
                    Simulator.Instance.agents_[agentNo].computeNeighbors();
                    Simulator.Instance.agents_[agentNo].computeNewVelocity();
                }

                doneEvent_.Set();
            }

            /**
             * <summary>updates the two-dimensional position and
             * two-dimensional velocity of each agent.</summary>
             *
             * <param name="obj">Unused.</param>
             */
            internal void update(object obj)
            {
                for (int agentNo = start_; agentNo < end_; ++agentNo)
                {
                    Simulator.Instance.agents_[agentNo].update();
                }

                doneEvent_.Set();
            }
        }

        internal IList<Agent> agents_;
        internal IList<Obstacle> obstacles_;
        internal KdTree kdTree_;
        internal float timeStep_;

        private static Simulator instance_ = new Simulator();

        private Agent defaultAgent_;
        private ManualResetEvent[] doneEvents_;
        private Worker[] workers_;
        private int numWorkers_;
        private float globalTime_;

        public static Simulator Instance
        {
            get
            {
                return instance_;
            }
        }

        /**
         * <summary>Adds a new agent with default properties to the simulation.
         * </summary>
         *
         * <returns>The number of the agent, or -1 when the agent defaults have
         * not been set.</returns>
         *
         * <param name="position">The two-dimensional starting position of this
         * agent.</param>
         */
        public int addAgent(Vector2 position)
        {
            if (defaultAgent_ == null)
            {
                return -1;
            }

            Agent agent = new Agent();
            agent.id_ = agents_.Count;
            agent.maxNeighbors_ = defaultAgent_.maxNeighbors_;
            agent.maxSpeed_ = defaultAgent_.maxSpeed_;
            agent.neighborDist_ = defaultAgent_.neighborDist_;
            agent.position_ = position;
            agent.radius_ = defaultAgent_.radius_;
            agent.timeHorizon_ = defaultAgent_.timeHorizon_;
            agent.timeHorizonObst_ = defaultAgent_.timeHorizonObst_;
            agent.velocity_ = defaultAgent_.velocity_;
            agents_.Add(agent);

            return agent.id_;
        }

        /**
         * <summary>Adds a new agent to the simulation.</summary>
         *
         * <returns>The number of the agent.</returns>
         *
         * <param name="position">The two-dimensional starting position of this
         * agent.</param>
         * <param name="neighborDist">The maximum distance (center point to
         * center point) to other agents this agent takes into account in the
         * navigation. The larger this number, the longer the running time of
         * the simulation. If the number is too low, the simulation will not be
         * safe. Must be non-negative.</param>
         * <param name="maxNeighbors">The maximum number of other agents this
         * agent takes into account in the navigation. The larger this number,
         * the longer the running time of the simulation. If the number is too
         * low, the simulation will not be safe.</param>
         * <param name="timeHorizon">The minimal amount of time for which this
         * agent's velocities that are computed by the simulation are safe with
         * respect to other agents. The larger this number, the sooner this
         * agent will respond to the presence of other agents, but the less
         * freedom this agent has in choosing its velocities. Must be positive.
         * </param>
         * <param name="timeHorizonObst">The minimal amount of time for which
         * this agent's velocities that are computed by the simulation are safe
         * with respect to obstacles. The larger this number, the sooner this
         * agent will respond to the presence of obstacles, but the less freedom
         * this agent has in choosing its velocities. Must be positive.</param>
         * <param name="radius">The radius of this agent. Must be non-negative.
         * </param>
         * <param name="maxSpeed">The maximum speed of this agent. Must be
         * non-negative.</param>
         * <param name="velocity">The initial two-dimensional linear velocity of
         * this agent.</param>
         */
        public int addAgent(Vector2 position, float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            Agent agent = new Agent();
            agent.id_ = agents_.Count;
            agent.maxNeighbors_ = maxNeighbors;
            agent.maxSpeed_ = maxSpeed;
            agent.neighborDist_ = neighborDist;
            agent.position_ = position;
            agent.radius_ = radius;
            agent.timeHorizon_ = timeHorizon;
            agent.timeHorizonObst_ = timeHorizonObst;
            agent.velocity_ = velocity;
            agents_.Add(agent);

            return agent.id_;
        }

        /**
         * <summary>Adds a new obstacle to the simulation.</summary>
         *
         * <returns>The number of the first vertex of the obstacle, or -1 when
         * the number of vertices is less than two.</returns>
         *
         * <param name="vertices">List of the vertices of the polygonal obstacle
         * in counterclockwise order.</param>
         *
         * <remarks>To add a "negative" obstacle, e.g. a bounding polygon around
         * the environment, the vertices should be listed in clockwise order.
         * </remarks>
         */
        public int addObstacle(IList<Vector2> vertices)
        {
            if (vertices.Count < 2)
            {
                return -1;
            }

            int obstacleNo = obstacles_.Count;

            for (int i = 0; i < vertices.Count; ++i)
            {
                Obstacle obstacle = new Obstacle();
                obstacle.point_ = vertices[i];

                if (i != 0)
                {
                    obstacle.previous_ = obstacles_[obstacles_.Count - 1];
                    obstacle.previous_.next_ = obstacle;
                }

                if (i == vertices.Count - 1)
                {
                    obstacle.next_ = obstacles_[obstacleNo];
                    obstacle.next_.previous_ = obstacle;
                }

                obstacle.direction_ = RVOMath.normalize(vertices[(i == vertices.Count - 1 ? 0 : i + 1)] - vertices[i]);

                if (vertices.Count == 2)
                {
                    obstacle.convex_ = true;
                }
                else
                {
                    obstacle.convex_ = (RVOMath.leftOf(vertices[(i == 0 ? vertices.Count - 1 : i - 1)], vertices[i], vertices[(i == vertices.Count - 1 ? 0 : i + 1)]) >= 0.0f);
                }

                obstacle.id_ = obstacles_.Count;
                obstacles_.Add(obstacle);
            }

            return obstacleNo;
        }

        /**
         * <summary>Clears the simulation.</summary>
         */
        public void Clear()
        {
            agents_ = new List<Agent>();
            defaultAgent_ = null;
            kdTree_ = new KdTree();
            obstacles_ = new List<Obstacle>();
            globalTime_ = 0.0f;
            timeStep_ = 0.1f;

            SetNumWorkers(0);
        }

        /**
         * <summary>Performs a simulation step and updates the two-dimensional
         * position and two-dimensional velocity of each agent.</summary>
         *
         * <returns>The global time after the simulation step.</returns>
         */
        public float doStep()
        {
            if (workers_ == null)
            {
                workers_ = new Worker[numWorkers_];
                doneEvents_ = new ManualResetEvent[workers_.Length];

                for (int block = 0; block < workers_.Length; ++block)
                {
                    doneEvents_[block] = new ManualResetEvent(false);
                    workers_[block] = new Worker(block * getNumAgents() / workers_.Length, (block + 1) * getNumAgents() / workers_.Length, doneEvents_[block]);
                }
            }

            kdTree_.buildAgentTree();

            for (int block = 0; block < workers_.Length; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers_[block].step);
            }

            WaitHandle.WaitAll(doneEvents_);

            for (int block = 0; block < workers_.Length; ++block)
            {
                doneEvents_[block].Reset();
                ThreadPool.QueueUserWorkItem(workers_[block].update);
            }

            WaitHandle.WaitAll(doneEvents_);

            globalTime_ += timeStep_;

            return globalTime_;
        }

        /**
         * <summary>Returns the specified agent neighbor of the specified agent.
         * </summary>
         *
         * <returns>The number of the neighboring agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose agent neighbor is
         * to be retrieved.</param>
         * <param name="neighborNo">The number of the agent neighbor to be
         * retrieved.</param>
         */
        public int getAgentAgentNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo].agentNeighbors_[neighborNo].Value.id_;
        }

        /**
         * <summary>Returns the maximum neighbor count of a specified agent.
         * </summary>
         *
         * <returns>The present maximum neighbor count of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * count is to be retrieved.</param>
         */
        public int getAgentMaxNeighbors(int agentNo)
        {
            return agents_[agentNo].maxNeighbors_;
        }

        /**
         * <summary>Returns the maximum speed of a specified agent.</summary>
         *
         * <returns>The present maximum speed of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose maximum speed is
         * to be retrieved.</param>
         */
        public float getAgentMaxSpeed(int agentNo)
        {
            return agents_[agentNo].maxSpeed_;
        }

        /**
         * <summary>Returns the maximum neighbor distance of a specified agent.
         * </summary>
         *
         * <returns>The present maximum neighbor distance of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * distance is to be retrieved.</param>
         */
        public float getAgentNeighborDist(int agentNo)
        {
            return agents_[agentNo].neighborDist_;
        }

        /**
         * <summary>Returns the count of agent neighbors taken into account to
         * compute the current velocity for the specified agent.</summary>
         *
         * <returns>The count of agent neighbors taken into account to compute
         * the current velocity for the specified agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose count of agent
         * neighbors is to be retrieved.</param>
         */
        public int getAgentNumAgentNeighbors(int agentNo)
        {
            return agents_[agentNo].agentNeighbors_.Count;
        }

        /**
         * <summary>Returns the count of obstacle neighbors taken into account
         * to compute the current velocity for the specified agent.</summary>
         *
         * <returns>The count of obstacle neighbors taken into account to
         * compute the current velocity for the specified agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose count of obstacle
         * neighbors is to be retrieved.</param>
         */
        public int getAgentNumObstacleNeighbors(int agentNo)
        {
            return agents_[agentNo].obstacleNeighbors_.Count;
        }

        /**
         * <summary>Returns the specified obstacle neighbor of the specified
         * agent.</summary>
         *
         * <returns>The number of the first vertex of the neighboring obstacle
         * edge.</returns>
         *
         * <param name="agentNo">The number of the agent whose obstacle neighbor
         * is to be retrieved.</param>
         * <param name="neighborNo">The number of the obstacle neighbor to be
         * retrieved.</param>
         */
        public int getAgentObstacleNeighbor(int agentNo, int neighborNo)
        {
            return agents_[agentNo].obstacleNeighbors_[neighborNo].Value.id_;
        }

        /**
         * <summary>Returns the ORCA constraints of the specified agent.
         * </summary>
         *
         * <returns>A list of lines representing the ORCA constraints.</returns>
         *
         * <param name="agentNo">The number of the agent whose ORCA constraints
         * are to be retrieved.</param>
         *
         * <remarks>The halfplane to the left of each line is the region of
         * permissible velocities with respect to that ORCA constraint.
         * </remarks>
         */
        public IList<Line> getAgentOrcaLines(int agentNo)
        {
            return agents_[agentNo].orcaLines_;
        }

        /**
         * <summary>Returns the two-dimensional position of a specified agent.
         * </summary>
         *
         * <returns>The present two-dimensional position of the (center of the)
         * agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * position is to be retrieved.</param>
         */
        public Vector2 getAgentPosition(int agentNo)
        {
            return agents_[agentNo].position_;
        }

        /**
         * <summary>Returns the two-dimensional preferred velocity of a
         * specified agent.</summary>
         *
         * <returns>The present two-dimensional preferred velocity of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * preferred velocity is to be retrieved.</param>
         */
        public Vector2 getAgentPrefVelocity(int agentNo)
        {
            return agents_[agentNo].prefVelocity_;
        }

        /**
         * <summary>Returns the radius of a specified agent.</summary>
         *
         * <returns>The present radius of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose radius is to be
         * retrieved.</param>
         */
        public float getAgentRadius(int agentNo)
        {
            return agents_[agentNo].radius_;
        }

        /**
         * <summary>Returns the time horizon of a specified agent.</summary>
         *
         * <returns>The present time horizon of the agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose time horizon is
         * to be retrieved.</param>
         */
        public float getAgentTimeHorizon(int agentNo)
        {
            return agents_[agentNo].timeHorizon_;
        }

        /**
         * <summary>Returns the time horizon with respect to obstacles of a
         * specified agent.</summary>
         *
         * <returns>The present time horizon with respect to obstacles of the
         * agent.</returns>
         *
         * <param name="agentNo">The number of the agent whose time horizon with
         * respect to obstacles is to be retrieved.</param>
         */
        public float getAgentTimeHorizonObst(int agentNo)
        {
            return agents_[agentNo].timeHorizonObst_;
        }

        /**
         * <summary>Returns the two-dimensional linear velocity of a specified
         * agent.</summary>
         *
         * <returns>The present two-dimensional linear velocity of the agent.
         * </returns>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * linear velocity is to be retrieved.</param>
         */
        public Vector2 getAgentVelocity(int agentNo)
        {
            return agents_[agentNo].velocity_;
        }

        /**
         * <summary>Returns the global time of the simulation.</summary>
         *
         * <returns>The present global time of the simulation (zero initially).
         * </returns>
         */
        public float getGlobalTime()
        {
            return globalTime_;
        }

        /**
         * <summary>Returns the count of agents in the simulation.</summary>
         *
         * <returns>The count of agents in the simulation.</returns>
         */
        public int getNumAgents()
        {
            return agents_.Count;
        }

        /**
         * <summary>Returns the count of obstacle vertices in the simulation.
         * </summary>
         *
         * <returns>The count of obstacle vertices in the simulation.</returns>
         */
        public int getNumObstacleVertices()
        {
            return obstacles_.Count;
        }

        /**
         * <summary>Returns the count of workers.</summary>
         *
         * <returns>The count of workers.</returns>
         */
        public int GetNumWorkers()
        {
            return numWorkers_;
        }

        /**
         * <summary>Returns the two-dimensional position of a specified obstacle
         * vertex.</summary>
         *
         * <returns>The two-dimensional position of the specified obstacle
         * vertex.</returns>
         *
         * <param name="vertexNo">The number of the obstacle vertex to be
         * retrieved.</param>
         */
        public Vector2 getObstacleVertex(int vertexNo)
        {
            return obstacles_[vertexNo].point_;
        }

        /**
         * <summary>Returns the number of the obstacle vertex succeeding the
         * specified obstacle vertex in its polygon.</summary>
         *
         * <returns>The number of the obstacle vertex succeeding the specified
         * obstacle vertex in its polygon.</returns>
         *
         * <param name="vertexNo">The number of the obstacle vertex whose
         * successor is to be retrieved.</param>
         */
        public int getNextObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].next_.id_;
        }

        /**
         * <summary>Returns the number of the obstacle vertex preceding the
         * specified obstacle vertex in its polygon.</summary>
         *
         * <returns>The number of the obstacle vertex preceding the specified
         * obstacle vertex in its polygon.</returns>
         *
         * <param name="vertexNo">The number of the obstacle vertex whose
         * predecessor is to be retrieved.</param>
         */
        public int getPrevObstacleVertexNo(int vertexNo)
        {
            return obstacles_[vertexNo].previous_.id_;
        }

        /**
         * <summary>Returns the time step of the simulation.</summary>
         *
         * <returns>The present time step of the simulation.</returns>
         */
        public float getTimeStep()
        {
            return timeStep_;
        }

        /**
         * <summary>Processes the obstacles that have been added so that they
         * are accounted for in the simulation.</summary>
         *
         * <remarks>Obstacles added to the simulation after this function has
         * been called are not accounted for in the simulation.</remarks>
         */
        public void processObstacles()
        {
            kdTree_.buildObstacleTree();
        }

        /**
         * <summary>Performs a visibility query between the two specified points
         * with respect to the obstacles.</summary>
         *
         * <returns>A boolean specifying whether the two points are mutually
         * visible. Returns true when the obstacles have not been processed.
         * </returns>
         *
         * <param name="point1">The first point of the query.</param>
         * <param name="point2">The second point of the query.</param>
         * <param name="radius">The minimal distance between the line connecting
         * the two points and the obstacles in order for the points to be
         * mutually visible (optional). Must be non-negative.</param>
         */
        public bool queryVisibility(Vector2 point1, Vector2 point2, float radius)
        {
            return kdTree_.queryVisibility(point1, point2, radius);
        }

        /**
         * <summary>Sets the default properties for any new agent that is added.
         * </summary>
         *
         * <param name="neighborDist">The default maximum distance (center point
         * to center point) to other agents a new agent takes into account in
         * the navigation. The larger this number, the longer he running time of
         * the simulation. If the number is too low, the simulation will not be
         * safe. Must be non-negative.</param>
         * <param name="maxNeighbors">The default maximum number of other agents
         * a new agent takes into account in the navigation. The larger this
         * number, the longer the running time of the simulation. If the number
         * is too low, the simulation will not be safe.</param>
         * <param name="timeHorizon">The default minimal amount of time for
         * which a new agent's velocities that are computed by the simulation
         * are safe with respect to other agents. The larger this number, the
         * sooner an agent will respond to the presence of other agents, but the
         * less freedom the agent has in choosing its velocities. Must be
         * positive.</param>
         * <param name="timeHorizonObst">The default minimal amount of time for
         * which a new agent's velocities that are computed by the simulation
         * are safe with respect to obstacles. The larger this number, the
         * sooner an agent will respond to the presence of obstacles, but the
         * less freedom the agent has in choosing its velocities. Must be
         * positive.</param>
         * <param name="radius">The default radius of a new agent. Must be
         * non-negative.</param>
         * <param name="maxSpeed">The default maximum speed of a new agent. Must
         * be non-negative.</param>
         * <param name="velocity">The default initial two-dimensional linear
         * velocity of a new agent.</param>
         */
        public void setAgentDefaults(float neighborDist, int maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector2 velocity)
        {
            if (defaultAgent_ == null)
            {
                defaultAgent_ = new Agent();
            }

            defaultAgent_.maxNeighbors_ = maxNeighbors;
            defaultAgent_.maxSpeed_ = maxSpeed;
            defaultAgent_.neighborDist_ = neighborDist;
            defaultAgent_.radius_ = radius;
            defaultAgent_.timeHorizon_ = timeHorizon;
            defaultAgent_.timeHorizonObst_ = timeHorizonObst;
            defaultAgent_.velocity_ = velocity;
        }

        /**
         * <summary>Sets the maximum neighbor count of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * count is to be modified.</param>
         * <param name="maxNeighbors">The replacement maximum neighbor count.
         * </param>
         */
        public void setAgentMaxNeighbors(int agentNo, int maxNeighbors)
        {
            agents_[agentNo].maxNeighbors_ = maxNeighbors;
        }

        /**
         * <summary>Sets the maximum speed of a specified agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose maximum speed is
         * to be modified.</param>
         * <param name="maxSpeed">The replacement maximum speed. Must be
         * non-negative.</param>
         */
        public void setAgentMaxSpeed(int agentNo, float maxSpeed)
        {
            agents_[agentNo].maxSpeed_ = maxSpeed;
        }

        /**
         * <summary>Sets the maximum neighbor distance of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose maximum neighbor
         * distance is to be modified.</param>
         * <param name="neighborDist">The replacement maximum neighbor distance.
         * Must be non-negative.</param>
         */
        public void setAgentNeighborDist(int agentNo, float neighborDist)
        {
            agents_[agentNo].neighborDist_ = neighborDist;
        }

        /**
         * <summary>Sets the two-dimensional position of a specified agent.
         * </summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * position is to be modified.</param>
         * <param name="position">The replacement of the two-dimensional
         * position.</param>
         */
        public void setAgentPosition(int agentNo, Vector2 position)
        {
            agents_[agentNo].position_ = position;
        }

        /**
         * <summary>Sets the two-dimensional preferred velocity of a specified
         * agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * preferred velocity is to be modified.</param>
         * <param name="prefVelocity">The replacement of the two-dimensional
         * preferred velocity.</param>
         */
        public void setAgentPrefVelocity(int agentNo, Vector2 prefVelocity)
        {
            agents_[agentNo].prefVelocity_ = prefVelocity;
        }

        /**
         * <summary>Sets the radius of a specified agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose radius is to be
         * modified.</param>
         * <param name="radius">The replacement radius. Must be non-negative.
         * </param>
         */
        public void setAgentRadius(int agentNo, float radius)
        {
            agents_[agentNo].radius_ = radius;
        }

        /**
         * <summary>Sets the time horizon of a specified agent with respect to
         * other agents.</summary>
         *
         * <param name="agentNo">The number of the agent whose time horizon is
         * to be modified.</param>
         * <param name="timeHorizon">The replacement time horizon with respect
         * to other agents. Must be positive.</param>
         */
        public void setAgentTimeHorizon(int agentNo, float timeHorizon)
        {
            agents_[agentNo].timeHorizon_ = timeHorizon;
        }

        /**
         * <summary>Sets the time horizon of a specified agent with respect to
         * obstacles.</summary>
         *
         * <param name="agentNo">The number of the agent whose time horizon with
         * respect to obstacles is to be modified.</param>
         * <param name="timeHorizonObst">The replacement time horizon with
         * respect to obstacles. Must be positive.</param>
         */
        public void setAgentTimeHorizonObst(int agentNo, float timeHorizonObst)
        {
            agents_[agentNo].timeHorizonObst_ = timeHorizonObst;
        }

        /**
         * <summary>Sets the two-dimensional linear velocity of a specified
         * agent.</summary>
         *
         * <param name="agentNo">The number of the agent whose two-dimensional
         * linear velocity is to be modified.</param>
         * <param name="velocity">The replacement two-dimensional linear
         * velocity.</param>
         */
        public void setAgentVelocity(int agentNo, Vector2 velocity)
        {
            agents_[agentNo].velocity_ = velocity;
        }

        /**
         * <summary>Sets the global time of the simulation.</summary>
         *
         * <param name="globalTime">The global time of the simulation.</param>
         */
        public void setGlobalTime(float globalTime)
        {
            globalTime_ = globalTime;
        }

        /**
         * <summary>Sets the number of workers.</summary>
         *
         * <param name="numWorkers">The number of workers.</param>
         */
        public void SetNumWorkers(int numWorkers)
        {
            numWorkers_ = numWorkers;

            if (numWorkers_ <= 0)
            {
                int completionPorts;
                ThreadPool.GetMinThreads(out numWorkers_, out completionPorts);
            }
            workers_ = null;
        }

        /**
         * <summary>Sets the time step of the simulation.</summary>
         *
         * <param name="timeStep">The time step of the simulation. Must be
         * positive.</param>
         */
        public void setTimeStep(float timeStep)
        {
            timeStep_ = timeStep;
        }

        /**
         * <summary>Constructs and initializes a simulation.</summary>
         */
        private Simulator()
        {
            Clear();
        }
    }
}
