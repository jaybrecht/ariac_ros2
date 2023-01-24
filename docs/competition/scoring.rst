Scoring
========

There are three major components of the Trial Score in ARIAC 2023.

  1. `Cost Factor`: How much does the system (sensors) cost?
  2. `Efficiency Factor`: How fast or efficiently did the system complete the task(s)
  3. `Completion Score`: How well did the task(s) get performed? Are all the correct parts there in the proper place?

Cost Factor
-----------

The Cost Factor :math:`CF` compares the cost of the sensors chosen by the team to the average of all sensor configurations across all teams.

  * :math:`TC` is the total cost of the sensors in the team's configuration.
  * :math:`TC_{avg}` is the average sensor cost across all teams.
  * :math:`w_c` is a weighting constant for cost factor.

  .. math::

    CF = w_c \cdot \frac{TC_{avg}}{TC}


Efficiency Factor
-----------------

The Efficiency Factor :math:`EF_i` for order :math:`i` compares the time to complete order :math:`i` for the team to the average of all teams's times to complete order :math:`i`.

  * :math:`T_i` is the time to complete order :math:`i`
  * :math:`T_{avg_{i}}` is the average time to complete order :math:`i` for all teams
  * :math:`w_t` is a weighting constant for efficiency factor.


  .. math::

    EF_i = w_t \cdot \frac{TC_{avg_{i}}}{T_i}


Completion Score
-----------------

Completion score varies between Kitting, Assembly, and Combined tasks. Each task is generated from Boolean conditions.

  Kitting Task Score
  -------------------

  * A kitting order has :math:`n` parts that need to be placed on the kitting tray.
  * A shipment has :math:`m` parts on the kitting tray.