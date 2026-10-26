# 功率限制说明文档

## 1 电机系统建模

<center>
<img src="assets/motor_model.png"/>
</center>

$$
\left\{
\begin{aligned}
&T=K_{t}i \\
&e=K_{e}\dot{\theta} \\
&J\ddot{\theta}+b\dot{\theta}=T \\
&L\dot{i}+Ri=V-e
\end{aligned}
\tag{1}
\right.
$$

在 SI 单位中，电机扭矩和反电动势常数相等，即 $K_t=K_e=K$，则电机控制方程如下

$$ 
\left\{
\begin{aligned}
&J\ddot{\theta}+b\dot{\theta}=Ki \\
&L\dot{i}+Ri=V-K\dot{\theta}
\end{aligned}
\tag{2}
\right.
$$

## 2 电机功率

$$
\begin{aligned}
P&=V\cdot{i} \\
&=Li\dot{i}+Ri^2+Ki\dot{\theta} \\
&\approx K_{i^2}i^2+K_{i\omega}i\omega
\end{aligned}
\tag{3}
$$

为了保证模型的统一性，式 (3) 中 $\omega$ 应取转子角速度。

按式 (3) 的模型进行测试后发现估计的功率与实际功率存在正比于 $\vert\dot{\theta}\vert$ 的稳态误差，引入修正项后得到电机功率模型

$$
P(i,\omega) \approx K_{i^2}i^2+K_{i\omega}i\omega+K_{\vert\omega\vert}\vert\omega\vert
\tag{4}
$$

## 3 功率限制思路

输入底盘所有电机的目标转速，反馈转速和底盘的目标功率，估计电机控制算法的电流，根据以上数据求解估计功率为目标功率时底盘电机转速与当前转速的比值 r 。将 r 限制在范围 [0,1] 后与底盘目标速度相乘得到限制功率后的底盘目标速度。

## 4 功率限制方程

功率方程

$$
P=\sum_{j}K_{i^2}I_j^2+\sum_{j}K_{i\omega}I_j(r_{m,j}\omega_j)+\sum_{j}K_{\vert\omega\vert}\vert r_{m,j}\omega_j \vert
\tag{5}=P_{ref}
$$

其中 $P_{ref}$ 为目标功率，$r_m$ 为底盘电机减速比

目标转速 $\boldsymbol{\omega}_{ref}$, 反馈转速 $\boldsymbol{\omega}_{fdb}$，功率限制后目标转速

$$
\begin{aligned}
\overline{\boldsymbol{\omega}}_{ref}=r\boldsymbol{\omega}_{ref} 
\end{aligned}
\tag{6}
$$

将电机PID控制器近似为比例控制器

$$
\begin{aligned}
I_j&\approx K_p(\overline{\omega}_{ref,j}-\omega_{fdb,j}) \\
 &=K_p(r\omega_{ref,j}-\omega_{fdb,j})
\end{aligned}
\tag{7}
$$

将式 (6)(7) 代入式 (5)，将底盘功率视为关于速度比例 r 的函数

$$
\begin{aligned}
P(r)=K_{i^2}K_p^2\sum_{j}(r\omega_{ref,j}-\omega_{fdb,j})^2+
K_{i\omega}\sum_{j}(r\omega_{ref,j}-\omega_{fdb,j})(r_{m,j}\omega_{fdb,j})+
K_{\vert\omega\vert}\sum_{j}\vert\omega_{fdb,j}\vert
\end{aligned}
\tag{8}
$$

化简式 (8)

$$
\begin{aligned}
P(r)=	\left(k_0\sum_{j}\omega_{ref,j}^2	\right)r^2+\left(k_1\sum_{j}\omega_{ref,j}\omega_{fdb,j}	\right)r+\left(k_2\sum_{j}\omega_{fdb,j}^2+k_3\sum_{j}\vert\omega_{fdb,j}\vert \right)
\end{aligned}
\tag{9}
$$

其中

$$
\begin{aligned}
&k_0=K_{i^2}K_p^2 \\
&k_1=-2K_{i^2}K_p^2+K_{i\omega}K_pr_m \\
&k_2=K_{i^2}K_p^2-K_{i\omega}K_pr_m \\
&k_3=K_{\vert\omega\vert}r_m \\
\end{aligned}
$$

方程 (5) 转化为求解

$$
P(r)=P_{ref} \rightarrow ar^2+br+c=0
\tag{10}
$$

其中

$$
\begin{aligned}
&a=k_0\sum_{j}\omega_{ref,j}^2 \\
&b=k_1\sum_{j}\omega_{ref,j}\omega_{fdb,j} \\
&c=k_2\sum_{j}\omega_{fdb,j}^2+k_3\sum_{j}\vert\omega_{fdb,j}\vert-P_{ref} \\
\end{aligned}
$$

至此功率限制已转化为式 (10) 的一元二次方程求解问题。

显然 $a\geq 0$ 。若 $a=0$ ，则 $\omega_{ref,j}=0$，此时 r 可任意取值，设定为1。

若 $a>0$ ，判别式 $\Delta=b^2-4ac$，

若 $P_{ref} \geq k_2\sum_{j}\omega_{fdb,j}^2+k_3\sum_{j}\vert\omega_{fdb,j}\vert$ 则有 $a>0,c\leq 0$，方程 (10) 必定有非负实根。

若 $P_{ref} < k_2\sum_{j}\omega_{fdb,j}^2+k_3\sum_{j}\vert\omega_{fdb,j}\vert$ 则有 $a>0,c>0$，方程 (10) 可能没有实根。注意到 $k_2\sum_{j}\omega_{fdb,j}^2+k_3\sum_{j}\vert\omega_{fdb,j}\vert$ 是关于 $\vert\omega_{fdb,j}\vert$ 的单调函数，所以对于反馈转速状态 $\boldsymbol{\omega}_{fdb}$，存在临界功率 $P_c(\vert\boldsymbol{\omega}_{fdb}\vert)$ 满足 $\Delta=0$。 $P_c$ 的物理意义为在转速状态 $\boldsymbol{\omega}_{fdb}$ 下，小于该功率的情况物理上不可能出现。

$$
r=\left\{
\begin{aligned}
&1 &a&=0，r 可取任意值 \\
&\frac{-b}{2a} &\Delta&<0，取临界功率 \\
&\frac{-b+\sqrt{\Delta}}{2a} &\Delta&\geq0，目标功率下速度比例上限 \\
\end{aligned}
\tag{11}
\right.
$$

## 5 功率限制实现

### 功率上限设置

$$
P_{ref}=P_{limit}+f_1(V_{cap})+f_2(E_{buffer})
\tag{12}
$$

其中 $P_{limit}$ 是裁判系统功率限制； $f_1(V_{cap})$ 为程序设置的从超级电容获取的功率，$V_{cap}$ 为超级电容电压，利用这一项对电容电压进行闭环控制，使电容电压不会过低；$E_{buffer}$ 为裁判系统能量缓冲，通过 $f_2(E_{buffer})$ 这一项对裁判系统缓冲能量进行闭环控制，在电压过低导致电容自带缓冲能量闭环失效的情况下防止能量缓冲继续降低。

### 速度设置

$$
\begin{aligned}
\overline{v}_x=\overline{r}v_x \\
\overline{v}_y=\overline{r}v_y \\
\overline{\omega}_z=\overline{r}\omega_z \\
\end{aligned}
\tag{13}
$$

其中

$$
\overline{r}=\left\{
\begin{aligned}
&r &0&\leq r \leq 1 \\
&0 &r&<0 \\
&1 &r&>1 \\
\end{aligned}
\right.
$$

## 参考资料

[1] [DC Motor Speed: System Modeling](https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling)

---

若公式无法显示可查看[图片](assets/power_limit.jpg)
