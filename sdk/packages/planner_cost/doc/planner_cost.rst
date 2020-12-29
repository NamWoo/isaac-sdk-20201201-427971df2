Planner Cost
===============

The Navigation local planner is based on the Linear Quadratic Regulator (LQR) planner. It works by
generating a trajectory that minimizes a cost function. Unfortunately, there is no single cost
function that works for all the applications; therefore, it's important to customize a cost
function for your own application. This is where PlannerCost and PlannerCostBuilder come into
action:

* PlannerCost provides an interface to generate a cost function:

.. code-block:: c++

    class PlannerCost {
     public:
      // Returns true if the current state is valid.
      virtual bool isValid(double time, const VectorXd& state);

      // Returns the evaluation at a given state and time.
      virtual double evaluate(double time, const VectorXd& state) = 0;

      // Adds the gradient of this cost function for the given state to the given vector `gradient`.
      // `gradient` uses a Ref<VectorXd> to allow block operation to passed to this function.
      virtual void addGradient(double time, const VectorXd& state,
                               Eigen::Ref<Eigen::VectorXd> gradient) = 0;

      // Adds the hessian of this cost function for the given state to the given matrix `hessian`.
      // `hessian` uses a Ref<VectorXd> to allow block operation to passed to this function.
      virtual void addHessian(double time, const VectorXd& state,
                              Eigen::Ref<Eigen::MatrixXd> hessian) = 0;
    };

* PlannerCostBuilder provides a component interface to add Planner cost to your
  application:

.. code-block:: c++

    class PlannerCostBuilder : public alice::Component {
     public:
      // Creates the cost function initially. Makes sure all necessary memory required for subsequent
      // calls to `update` is allocated.
      virtual PlannerCost* build() = 0;

      // Prepares the cost function for the given time interval.
      // Does not do any dynamic memory allocations.
      virtual void update(double start_time, double end_time) {}

      // Destroys the cost function and all memory which was allocated during `build`.
      virtual void destroy() = 0;

      // Returns a pointer to the maintained cost function
      virtual PlannerCost* get() = 0;
    };

Components
----------

The main component is :ref:`isaac.planner_cost.PlannerCostBuilder`, which is the interface to add
a new PlannerCost.

The following are cost functions that are already implemented in Isaac SDK and ready for use:

* :ref:`isaac.planner_cost.AdditionBuilder`: Used to add several costs together.
* :ref:`isaac.planner_cost.SmoothMinimumBuilder`: Used to compute the minimum of a list
  of costs. It is not the exact minimum, but an approximation that is differentiable.
* :ref:`isaac.planner_cost.RangeConstraintsCostBuilder`: Helps you to create a quadratic
  cost based on the state. You can define a minimum and maximum value, and if the state falls
  outside this range, a quadratic cost will be created.
* :ref:`isaac.planner_cost.DistanceQuadraticCostBuilder`: Expects a PlannerCost to return a
  distance. It will create a quadratic cost based on a target distance and the actual distance.
* :ref:`isaac.planner_cost.ObstacleDistanceBuilder`: Returns the signed distance to an obstacle.
  It is not a cost function in itself, but it can be used with
  :ref:`isaac.planner_cost.DistanceQuadraticCostBuilder` to create a quadratic cost function.
* :ref:`isaac.planner_cost.CirclesUnionSmoothDistanceBuilder`: This is an helper function to call
  another PlannerCost function for all the circles of the RobotShape.

Getting Started
---------------

You can run Flatsim to see how the Navigation local planner performs:

.. code-block:: bash

   bazel run //packages/flatsim/apps:flatsim -- --demo demo_1

If you want to create your own cost, you should first determine whether the existing costs in
:code:`packages/planner_cost/gems` do what you need. If none of these costs are sufficient, you
will need to first create a class implementing the PlannerCost interface.

For example, let's look at ScalarMultiplication, which takes a PlannerCost as input and
multiplies it by a constant:

.. code-block:: c++

    // This is an implementation of PlannerCost.
    // It takes another PlannerCost and simply multiplies by a constant value.
    class ScalarMultiplication : public PlannerCost {
     public:
      ScalarMultiplication(PlannerCost* cost, double constant) : cost_(cost), constant_(constant) {}

      // Returns true if the current state is valid. Here we will just rely on the other PlannerCost
      bool isValid(double time, const VectorXd& state) override {
        return cost_->isValid(time, state);
      }

      // Returns the evaluation at a given state and time.
      // We can multiply the result of cost_->evaluate() by our constant.
      double evaluate(double time, const VectorXd& state) override {
        return constant_ * cost_->evaluate(time, state);
      }

      // Adds the gradient of this cost function for the given state to the given vector `gradient`.
      // `gradient` uses a Ref<VectorXd> to allow block operation to passed to this function.
      // We need to scale the gradient by our constant.
      void addGradient(double time, const VectorXd& state, Eigen::Ref<VectorXd> gradient) override {
        VectorXd tmp_gradient = VectorXd::Zero(gradient.size());
        cost_->addGradient(time, state, tmp_gradient);
        gradient += tmp_gradient * constant_;
      }

      // Adds the hessian of this cost function for the given state to the given matrix `hessian`.
      // `hessian` uses a Ref<VectorXd> to allow block operation to passed to this function.
      // We need to scale the hessian by our constant.
      void addHessian(double time, const VectorXd& state, Eigen::Ref<MatrixXd> hessian) override {
        MatrixXd tmp_hessian = MatrixXd::Zero(hessian.rows(), hessian.cols());
        cost_->addHessian(time, state, tmp_hessian);
        hessian += tmp_hessian * constant_;
      }

     private:
      // Hold another cost_
      PlannerCost* cost_ = nullptr;
      double constant_ = 1.0;
    };

Once you have your new PlannerCost, you can use a custom builder, as shown below. Note that it
must implement the interface PlannerCostBuilder:

.. code-block:: c++

    class ScalarMultiplicationBuilder : public PlannerCostBuilder {
     public:
      // Creates the cost function initially. Makes sure all necessary memory required for subsequent
      // calls to `update` is allocated.
      PlannerCost* build() override {
        builder_ = node()->app()->findComponentByName<PlannerCostBuilder>(get_component_name());
        ASSERT(builder_ != nullptr,
               "Failed to load the component: %s", get_component_name().c_str());
        cost_.reset(new ScalarMultiplication(builder_->build(), get_constant()));
        return static_cast<PlannerCost*>(cost_.get());
      }

      // Prepares the cost function for the given time interval.
      // Does not do any dynamic memory allocations.
      void update(double start_time, double end_time) override {
        builder_->update(start_time, end_time);
      }

      // Destroys the cost function and all memory which was allocated during `build`.
      void destroy() override {
        cost_.reset();
        builder_->destroy();
      }

      // Returns a pointer to the maintained cost function
      PlannerCost* get() override {
        return static_cast<PlannerCost*>(cost_.get());
      }

      // Name of the component implementating a PlannerCostBuilder to be used as distance function
      ISAAC_PARAM(std::string, component_name);

      // Constant multiplication factor
      ISAAC_PARAM(double, constant, 20.0);

     private:
      std::unique_ptr<ScalarMultiplication> cost_;
      PlannerCostBuilder* builder_;
    };

We now have a new PlannerCost that we can use to scale any existing PlannerCost. We also have a
builder for it. In the next section, we will look at how to expand the existing navigation graph to
scale an existing cost.

Customizing the Cost via the Application Graph
----------------------------------------------

To customize the graph, edit the :code:`packages/navigation/apps/differential_base_control.subgraph.json`
file.

First you should locate the Node containing all the builders:

.. code::

    {
      "name": "lqr_state_cost",
      "components": [
        {
          "name": "TotalSum",
          "type": "isaac::planner_cost::AdditionBuilder"
        },
        {
          "name": "LimitRange",
          "type": "isaac::planner_cost::RangeConstraintsCostBuilder"
        },
        {
          "name": "TargetRange",
          "type": "isaac::planner_cost::RangeConstraintsCostBuilder"
        },
        {
          "name": "SmoothMinimumBuilder",
          "type": "isaac::planner_cost::SmoothMinimumBuilder"
        },
        {
          "name": "CirclesUnionSmoothDistanceBuilder",
          "type": "isaac::planner_cost::CirclesUnionSmoothDistanceBuilder"
        },
        {
          "name": "ObstacleLocalMap",
          "type": "isaac::planner_cost::ObstacleDistanceBuilder"
        },
        {
          "name": "ObstacleRestrictedArea",
          "type": "isaac::planner_cost::ObstacleDistanceBuilder"
        },
        {
          "name": "DistanceQuadraticCostBuilder",
          "type": "isaac::planner_cost::DistanceQuadraticCostBuilder"
        }
      ]
    },
    {
      "name": "lqr_control_cost",
      "components": [
        {
          "name": "RangeConstraintsCostBuilder",
          "type": "isaac::planner_cost::RangeConstraintsCostBuilder"
        }
      ]
    },


:code:`lqr_state_cost` contains the list of  builders used to compute the cost associated with
the states along the trajectory, while :code:`lqr_control_cost` contains the cost associated with
the control.

Further down, you can find the config parameter associated with these costs:

.. code::

    "lqr": {
      "isaac.lqr.DifferentialBaseLqrPlanner": {
        ...
        "state_planner_cost_name": "$(fullname lqr_state_cost/TotalSum)",
        "control_planner_cost_name": "$(fullname lqr_control_cost/RangeConstraintsCostBuilder)"
        ...
      }
    },

Here we define the root of the cost associated with the controls and the root associated with the
states:

* For the controls, we have a single cost of type :ref:`isaac.planner_cost.RangeConstraintsCostBuilder`

* For the states, the root is of type :ref:`isaac.planner_cost.AdditionBuilder`, which means we will
  be adding a list of cost. Looking at the config of :code:`TotalSum`, we can find which costs are
  added:

.. code::

    "TotalSum": {
      "component_names": [
        "$(fullname lqr_state_cost/DistanceQuadraticCostBuilder)",
        "$(fullname lqr_state_cost/LimitRange)",
        "$(fullname lqr_state_cost/TargetRange)"
      ]
    },

There are three costs added to compute the final cost:

* Two of them are of type :ref:`isaac.planner_cost.RangeConstraintsCostBuilder`.

* The last one is of type :ref:`isaac.planner_cost.DistanceQuadraticCostBuilder`. This is another
  recursive call, which depends on another Builder of type
  :ref:`isaac.planner_cost.CirclesUnionSmoothDistanceBuilder`, which itself depends on the Builder
  of type :ref:`isaac.planner_cost.SmoothMinimumBuilder`, which computes the minimum value of a list
  of :ref:`isaac.planner_cost.ObstacleDistanceBuilder`:

.. code::

    "DistanceQuadraticCostBuilder": {
      "component_name": "$(fullname lqr_state_cost/CirclesUnionSmoothDistanceBuilder)"
    },

    "CirclesUnionSmoothDistanceBuilder": {
      "component_name": "$(fullname lqr_state_cost/SmoothMinimumBuilder)"
    },

    "SmoothMinimumBuilder": {
      "component_names": [
        "$(fullname lqr_state_cost/ObstacleLocalMap)",
        "$(fullname lqr_state_cost/ObstacleRestrictedArea)"
      ]
    },

    "ObstacleLocalMap": {
      "obstacle_name": "local_map"
    },
    "ObstacleRestrictedArea": {
      "obstacle_name": "map/restricted_area"
    },

This can look complicated at first--let's analyze it starting from the end:

* :code:`ObstacleLocalMap` and :code:`ObstacleRestrictedArea` are both loading an obstacle from
  Atlas and return the signed distance from a 2d to an obstacle.

* :code:`SmoothMinimumBuilder` helps approximate the minimum distance--ultimately, we want
  to know how close the robot is to the closest obstacle. If you need to handle more obstacles, this
  would be a good place to make additions.

* :code:`CirclesUnionSmoothDistanceBuilder` is an helper function that helps compute the distance,
  not only for a single 2d point, but for all the circles in the :code:`SphericalRobotShape`. It
  will return the distance of the robot from a list of obstacles.

* Finally :ref:`isaac.planner_cost.DistanceQuadraticCostBuilder` expects a distance function and
  computes the cost: :math:`0.5 * gain * min(0, distance(state) - target_distance - alpha * speed)^2`.
  We simply pass the distance function computed by :code:`CirclesUnionSmoothDistanceBuilder`.


Let's explore how you can modify the above example to add your custom cost function. Assume you
have the following:

* A new :code:`CustomDistanceBuilder` that returns a distance to some obstacles, but in centimeters.

* The :code:`ScalarMultiplicationBuilder` we have defined above.

Now we need to combine both to compute the distance in meters, and we need to add it to the list of
obstacles. First we need to add both components to our node:


.. code::

    {
      "name": "lqr_state_cost",
      "components": [
        {
          "name": "TotalSum",
          "type": "isaac::planner_cost::AdditionBuilder"
        },
        ...
        {
          "name": "DistanceQuadraticCostBuilder",
          "type": "isaac::planner_cost::DistanceQuadraticCostBuilder"
        },
        {
          "name": "ScalarMultiplicationBuilder",
          "type": "isaac::planner_cost::ScalarMultiplicationBuilder"
        },
        {
          "name": "CustomDistanceBuilder",
          "type": "isaac::planner_cost::CustomDistanceBuilder"
        }
      ]
    },

Afterward, we need create the config for them:

.. code::

    "lqr_state_cost": {
      ...
      "ScalarMultiplicationBuilder": {
        "component_name": "$(fullname lqr_state_cost/CustomDistanceBuilder)",
        "constant": 100.0
      },
      "CustomDistanceBuilder": {
        ...
      }
    }

Finally, we need to add our new distance to the list of existing obstacles:

.. code::

    "SmoothMinimumBuilder": {
      "component_names": [
        "$(fullname lqr_state_cost/ObstacleLocalMap)",
        "$(fullname lqr_state_cost/ObstacleRestrictedArea)",
        "$(fullname lqr_state_cost/ScalarMultiplicationBuilder)"
      ]
    },

We have successfully added a new obstacle using a custom builder.
