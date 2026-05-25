# Impedance Control
Este es el repositorio de Lab Session 4: Impedance Control.

## Ejecución 

```bash
ros2 launch uma_arm_description uma_arm_visualization.launch.py
ros2 launch impedance_control uma_arm_dynamics_launch.py
ros2 launch impedance_control gravity_compensation_launch.py
ros2 launch impedance_control dynamics_cancellation_launch.py
ros2 launch impedance_control pd_controller_launch.py
ros2 launch impedance_control impedance_controller_launch.py
```



## Explicación Detallada de los Cambios en el Controlador de Impedancia

A continuación se detallan las cinco funciones modificadas en el archivo `impedance_controller.cpp`, relacionando la formulación matemática con su implementación en C++.

---

### 1. Cinemática Directa (`forward_kinematics`)

**Objetivo:** Calcular la posición cartesiana actual del efector final, $\mathbf{x}$, a partir de las posiciones articulares actuales, $\mathbf{q}$.

**Formulación Matemática:**
La cinemática directa para un manipulador planar 2R se define como:
$$ \mathbf{x} = \begin{bmatrix} x \\ y \end{bmatrix} = \begin{bmatrix} l_1 \cos(q_1) + l_2 \cos(q_1 + q_2) \\ l_1 \sin(q_1) + l_2 \sin(q_1 + q_2) \end{bmatrix} $$

**Implementación en C++:**
Se extraen las variables articulares $q_1$ y $q_2$ del vector `joint_positions_` y se aplican las ecuaciones trigonométricas utilizando las longitudes de los eslabones $l_1$ (`l1_`) y $l_2$ (`l2_`).

```cpp
    Eigen::VectorXd forward_kinematics()
    {
        Eigen::VectorXd x(2);
        double q1 = joint_positions_(0);
        double q2 = joint_positions_(1);

        x(0) = l1_ * cos(q1) + l2_ * cos(q1 + q2);
        x(1) = l1_ * sin(q1) + l2_ * sin(q1 + q2);

        return x;
    }
```

---

### 2. Jacobiano y su Derivada (`update_jacobians`)

**Objetivo:** Calcular la matriz Jacobiana analítica, $\mathbf{J}(\mathbf{q})$, y su derivada temporal, $\dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})$, necesarias para las transformaciones de velocidad y aceleración entre el espacio articular y el cartesiano.

**Formulación Matemática:**
El Jacobiano se obtiene derivando la cinemática directa respecto a $\mathbf{q}$:
$$ \mathbf{J}(\mathbf{q}) = \begin{bmatrix} -l_1\sin(q_1) - l_2\sin(q_1+q_2) & -l_2\sin(q_1+q_2) \\ l_1\cos(q_1) + l_2\cos(q_1+q_2) & l_2\cos(q_1+q_2) \end{bmatrix} $$

La derivada del Jacobiano respecto al tiempo requiere aplicar rigurosamente la regla de la cadena (corrigiendo la errata del documento original, ya que derivar $\sin(q_1+q_2)$ introduce el factor $(\dot{q}_1 + \dot{q}_2)$):
$$ \dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}}) = \begin{bmatrix} -l_1\cos(q_1)\dot{q}_1 - l_2\cos(q_1+q_2)(\dot{q}_1+\dot{q}_2) & -l_2\cos(q_1+q_2)(\dot{q}_1+\dot{q}_2) \\ -l_1\sin(q_1)\dot{q}_1 - l_2\sin(q_1+q_2)(\dot{q}_1+\dot{q}_2) & -l_2\sin(q_1+q_2)(\dot{q}_1+\dot{q}_2) \end{bmatrix} $$

**Implementación en C++:**
Se precalculan los términos trigonométricos para optimizar el coste computacional y se asignan a las matrices correspondientes de Eigen.

```cpp
    void update_jacobians()
    {
        double q1 = joint_positions_(0);
        double q2 = joint_positions_(1);
        double dq1 = joint_velocities_(0);
        double dq2 = joint_velocities_(1);

        double s1 = sin(q1);
        double c1 = cos(q1);
        double s12 = sin(q1 + q2);
        double c12 = cos(q1 + q2);

        // Calculate J(q)
        jacobian_ << -l1_ * s1 - l2_ * s12, -l2_ * s12,
                      l1_ * c1 + l2_ * c12,  l2_ * c12;

        // Calculate J'(q,q') 
        jacobian_derivative_ << -l1_ * c1 * dq1 - l2_ * c12 * (dq1 + dq2), -l2_ * c12 * (dq1 + dq2),
                                -l1_ * s1 * dq1 - l2_ * s12 * (dq1 + dq2), -l2_ * s12 * (dq1 + dq2);

        RCLCPP_INFO(this->get_logger(), "Jacobian:\n[%.3f, %.3f]\n[%.3f, %.3f]",
                    jacobian_(0, 0), jacobian_(0, 1),
                    jacobian_(1, 0), jacobian_(1, 1));

        double det = jacobian_.determinant();
        RCLCPP_INFO(this->get_logger(), "Jacobian determinant: %.6f", det);
    }
```

---

### 3. Cinemática Diferencial de Primer Orden (`differential_kinematics`)

**Objetivo:** Calcular la velocidad cartesiana actual del efector final, $\dot{\mathbf{x}}$.

**Formulación Matemática:**
La relación entre las velocidades articulares, $\dot{\mathbf{q}}$, y las velocidades cartesianas se define mediante el Jacobiano:
$$ \dot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\dot{\mathbf{q}} $$

**Implementación en C++:**
Consiste en una multiplicación matricial directa utilizando las variables previamente calculadas/actualizadas.

```cpp
    Eigen::MatrixXd differential_kinematics()
    {
        Eigen::VectorXd x_dot = jacobian_ * joint_velocities_;
        return x_dot;
    }
```

---

### 4. Controlador de Impedancia (`impedance_controller`)

**Objetivo:** Determinar la aceleración cartesiana deseada, $\ddot{\mathbf{x}}_d$, que el efector final debe seguir para emular el comportamiento masa-muelle-amortiguador frente a fuerzas externas.

**Formulación Matemática:**
Se definen los errores de posición y velocidad respecto al punto de equilibrio:
$$ \tilde{\mathbf{x}} = \mathbf{x} - \mathbf{x}_d $$
$$ \dot{\tilde{\mathbf{x}}} = \dot{\mathbf{x}} - \dot{\mathbf{x}}_d $$

La ecuación de la impedancia de segundo orden establece la relación de fuerzas:
$$ \mathbf{M}\ddot{\mathbf{x}}_d + \mathbf{B}\dot{\tilde{\mathbf{x}}} + \mathbf{K}\tilde{\mathbf{x}} = \mathbf{f}_{ext} $$

Despejando la aceleración deseada $\ddot{\mathbf{x}}_d$:
$$ \ddot{\mathbf{x}}_d = \mathbf{M}^{-1}(\mathbf{f}_{ext} - \mathbf{B}\dot{\tilde{\mathbf{x}}} - \mathbf{K}\tilde{\mathbf{x}}) $$

**Implementación en C++:**
Se asume una velocidad de equilibrio nula ($\dot{\mathbf{x}}_d = \mathbf{0}$). Se calculan los errores y se despeja la aceleración invirtiendo la matriz de masa cartesianas (`mass_matrix_.inverse()`).

```cpp
    Eigen::VectorXd impedance_controller()
    {
        Eigen::VectorXd x_dot_d = Eigen::VectorXd::Zero(2); 

        Eigen::VectorXd x_error = cartesian_pose_ - equilibrium_pose_;
        Eigen::VectorXd x_dot_error = cartesian_velocities_ - x_dot_d;

        Eigen::VectorXd x_ddot = mass_matrix_.inverse() * (external_wrenches_ - stiffness_matrix_ * x_error - damping_matrix_ * x_dot_error);

        return x_ddot;
    }
```

---

### 5. Aceleraciones Articulares Deseadas (`calculate_desired_joint_accelerations`)

**Objetivo:** Convertir la aceleración cartesiana de control, $\ddot{\mathbf{x}}_d$, de vuelta al espacio articular para obtener $\ddot{\mathbf{q}}$, que es lo que el sistema dinámico de nivel inferior requiere para cancelar las dinámicas y aplicar los torques.

**Formulación Matemática:**
La cinemática diferencial de segundo orden se define como:
$$ \ddot{\mathbf{x}} = \mathbf{J}(\mathbf{q})\ddot{\mathbf{q}} + \dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}} $$

Despejando el vector de aceleraciones articulares, $\ddot{\mathbf{q}}$:
$$ \ddot{\mathbf{q}} = \mathbf{J}(\mathbf{q})^{-1}[\ddot{\mathbf{x}} - \dot{\mathbf{J}}(\mathbf{q}, \dot{\mathbf{q}})\dot{\mathbf{q}}] $$

**Implementación en C++:**
Se invierte el Jacobiano (`jacobian_.inverse()`) y se evalúa la expresión con la aceleración cartesiana calculada en la función anterior.

```cpp
    Eigen::VectorXd calculate_desired_joint_accelerations()
    {
        RCLCPP_INFO(this->get_logger(), "x_ddot: [%.3f, %.3f]",
                    desired_cartesian_accelerations_(0), desired_cartesian_accelerations_(1));

        Eigen::VectorXd q_ddot = jacobian_.inverse() * (desired_cartesian_accelerations_ - jacobian_derivative_ * joint_velocities_);

        return q_ddot;
    }
```


## Experiment 1: Apply virtual forces to the robot



## Experiment 2: Change the equilibrium pose

