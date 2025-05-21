import math
import matplotlib.pyplot as plt
import numpy as np

def calcular_parametros(V, steering_degree, ax=2):
    Mv = 1500
    L = 2.8
    Lr = 1.2
    g = 9.81
    a1 = 1.4
    a2 = 1.4
    h = 1.0
    Ca = 60000

    steering_ratio = 20
    delta = steering_degree / steering_ratio
    delta_rad = math.radians(delta)

    wf = (Mv * g * a2) / L - (Mv * ax * h) / L
    wr = (Mv * g * a1) / L + (Mv * ax * h) / L

    Mf = wf / g
    Mr = wr / g

    if abs(delta_rad) < 1e-6:
        R = float('inf')
    else:
        R = L / math.tan(delta_rad)

    ay = V**2 / R if R != float('inf') else 0

    wfi = wf / 2 - (Mf * ay * h) / Lr
    wfo = wf / 2 + (Mf * ay * h) / Lr
    wri = wr / 2 - (Mr * ay * h) / Lr
    wro = wr / 2 + (Mr * ay * h) / Lr

    mf = (wfi + wfo) / g
    mr = (wri + wro) / g

    if V == 0 or R == float('inf'):
        yaw_rate = 0
    else:
        yaw_rate = V / (L + ((mf - mr) / Ca) * V**2)

    yaw_rate_deg = math.degrees(yaw_rate)

    vx = V
    # vy vai ser integrado externamente no trajeto C

    if vx != 0:
        beta_rad = math.atan((yaw_rate * a2) / vx)  # só para slip angle
    else:
        beta_rad = 0
    beta_deg = math.degrees(beta_rad)

    return {
        "Velocidade": V,
        "Raio": R,
        "ay": ay,
        "ax": ax,
        "Yaw Rate (rad/s)": yaw_rate,
        "Yaw Rate (deg/s)": yaw_rate_deg,
        "vx": vx,
        "Slip Angle (deg)": beta_deg,
        "wfi": wfi,
        "wfo": wfo,
        "wri": wri,
        "wro": wro,
    }

def simular_trajeto(tipo_trajeto):
    L = 2.8
    steering_ratio = 20
    steering_rate = 5  # graus/s
    ax = 2
    V_max = 60 / 3.6

    if tipo_trajeto == 'A':
        print("Simulacro Trajeto A: condução em linha reta até atingir 60km/h.")

        dt = 0.1
        vy_const = 0
        steering_profile = [0] * 100  # Direção constante (reta)

        x, y, theta = 0.0, 0.0, 0.0
        vx = 0.0
        vy = 0.0
        x_list, y_list = [], []
        tempos, vx_list, vy_list, ay_list, ax_list = [], [], [], [], []
        yaw_list, slip_list = [], []
        wfi_list, wfo_list, wri_list, wro_list = [], [], [], []
        t = 0.0

        for steering in steering_profile:
            if vx < V_max:
                vx += ax * dt
                if vx > V_max:
                    vx = V_max

            delta_rad = math.radians(steering / steering_ratio)
            R = L / math.tan(delta_rad) if abs(delta_rad) > 1e-6 else float('inf')
            omega = vx / R if R != float('inf') else 0

            theta += omega * dt
            x += vx * math.cos(theta) * dt
            y += vy_const * math.sin(theta) * dt

            res = calcular_parametros(vx, steering, ax=ax)

            tempos.append(t)
            x_list.append(x)
            y_list.append(y)
            vx_list.append(res["vx"])
            vy_list.append(vy)
            ax_list.append(res["ax"])
            ay_list.append(res["ay"])
            yaw_list.append(res["Yaw Rate (deg/s)"])
            slip_list.append(res["Slip Angle (deg)"])
            wfi_list.append(res["wfi"])
            wfo_list.append(res["wfo"])
            wri_list.append(res["wri"])
            wro_list.append(res["wro"])

            t += dt

        # === Gráficos ===
        plt.figure(figsize=(14, 14))

        plt.subplot(6, 1, 1)
        plt.plot(tempos, vx_list, label="vx (m/s)")
        plt.plot(tempos, vy_list, label="vy (m/s)")
        plt.ylabel("Velocidade (m/s)")
        plt.title("Velocidades")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 2)
        plt.plot(tempos, ax_list, label="ax (m/s²)")
        plt.plot(tempos, ay_list, label="ay (m/s²)")
        plt.ylabel("Aceleração (m/s²)")
        plt.title("Acelerações")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 3)
        plt.plot(tempos, yaw_list, label="Yaw Rate (°/s)", color="green")
        plt.ylabel("Yaw Rate")
        plt.title("Velocidade Angular")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 4)
        plt.plot(tempos, slip_list, label="Slip Angle (°)", color="purple")
        plt.ylabel("Slip Angle")
        plt.title("Ângulo de Escorregamento")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 5)
        plt.plot(tempos, wfi_list, label="wfi")
        plt.plot(tempos, wfo_list, label="wfo")
        plt.plot(tempos, wri_list, label="wri")
        plt.plot(tempos, wro_list, label="wro")
        plt.ylabel("Peso (N)")
        plt.xlabel("Tempo (s)")
        plt.title("Distribuição de Peso nas Rodas")
        plt.grid(True)
        plt.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.suptitle("Simulação do Trajeto A", fontsize=14)
        plt.show()

        plt.figure(figsize=(8, 6))
        plt.plot(x_list, y_list, label='Trajetória XY', color='blue')
        plt.xlabel("Posição X (m)")
        plt.ylabel("Posição Y (m)")
        plt.title("Posição do Veículo (Trajetória XY)")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()
        return

    if tipo_trajeto == 'B':
        print("Simulacro Trajeto B: condução em curva com um angulo de volante constante de -100º até atingir a velocidade de 60km/h.")

        dt = 0.1
        steering = -100  # Volante travado
        delta_rad = math.radians(steering / steering_ratio)
        R = L / math.tan(delta_rad) if abs(delta_rad) > 1e-6 else float('inf')

        x, y, theta = 0.0, 0.0, 0.0
        vx = 0.0
        vy = 0.0
        x_list, y_list = [], []
        tempos, vx_list, vy_list, ay_list, ax_list = [], [], [], [], []
        yaw_list, slip_list = [], []
        wfi_list, wfo_list, wri_list, wro_list = [], [], [], []
        t = 0.0

        while vx < V_max:
            vx += ax * dt
            if vx > V_max:
                vx = V_max

            omega = vx / R if R != float('inf') else 0
            theta += omega * dt
            x += vx * math.cos(theta) * dt
            y += vx * math.sin(theta) * dt

            res = calcular_parametros(vx, steering, ax=ax)

            tempos.append(t)
            x_list.append(x)
            y_list.append(y)
            vx_list.append(res["vx"])
            vy_list.append(vy)
            ax_list.append(res["ax"])
            ay_list.append(res["ay"])
            yaw_list.append(res["Yaw Rate (deg/s)"])
            slip_list.append(res["Slip Angle (deg)"])
            wfi_list.append(res["wfi"])
            wfo_list.append(res["wfo"])
            wri_list.append(res["wri"])
            wro_list.append(res["wro"])

            t += dt

        # === Gráficos ===
        plt.figure(figsize=(14, 14))

        plt.subplot(6, 1, 1)
        plt.plot(tempos, vx_list, label="vx (m/s)")
        plt.plot(tempos, vy_list, label="vy (m/s)")
        plt.ylabel("Velocidade (m/s)")
        plt.title("Velocidades")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 2)
        plt.plot(tempos, ax_list, label="ax (m/s²)")
        plt.plot(tempos, ay_list, label="ay (m/s²)")
        plt.ylabel("Aceleração (m/s²)")
        plt.title("Acelerações")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 3)
        plt.plot(tempos, yaw_list, label="Yaw Rate (°/s)", color="green")
        plt.ylabel("Yaw Rate")
        plt.title("Velocidade Angular")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 4)
        plt.plot(tempos, slip_list, label="Slip Angle (°)", color="purple")
        plt.ylabel("Slip Angle")
        plt.title("Ângulo de Escorregamento")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 5)
        plt.plot(tempos, wfi_list, label="wfi")
        plt.plot(tempos, wfo_list, label="wfo")
        plt.plot(tempos, wri_list, label="wri")
        plt.plot(tempos, wro_list, label="wro")
        plt.ylabel("Peso (N)")
        plt.xlabel("Tempo (s)")
        plt.title("Distribuição de Peso nas Rodas")
        plt.grid(True)
        plt.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.suptitle("Simulação do Trajeto B", fontsize=14)
        plt.show()

        plt.figure(figsize=(8, 6))
        plt.plot(x_list, y_list, label='Trajetória XY', color='blue')
        plt.xlabel("Posição X (m)")
        plt.ylabel("Posição Y (m)")
        plt.title("Posição do Veículo (Trajetória XY)")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()
        return

    if tipo_trajeto == 'C':
        print("Simulacro Trajeto C com direção variável a 20 km/h.")

        dt = 0.1
        v_const = 20 / 3.6
        steering_targets = [0, -150, 0, 150, 0]
        steering_profile = []

        for i in range(len(steering_targets) - 1):
            start = steering_targets[i]
            end = steering_targets[i + 1]
            direction = np.sign(end - start)
            delta_total = abs(end - start)
            duration = delta_total / steering_rate
            steps = int(duration / dt)
            for s in range(steps):
                angle = start + direction * s * steering_rate * dt
                steering_profile.append(angle)
        steering_profile.append(steering_targets[-1])

        x, y, theta = 0.0, 0.0, 0.0
        vy = 0.0  # inicialização de vy
        x_list, y_list = [], []
        tempos, vx_list, vy_list, ay_list, ax_list = [], [], [], [], []
        yaw_list, slip_list = [], []
        wfi_list, wfo_list, wri_list, wro_list = [], [], [], []
        t = 0.0

        for steering in steering_profile:
            delta_rad = math.radians(steering / steering_ratio)
            R = L / math.tan(delta_rad) if abs(delta_rad) > 1e-6 else float('inf')
            omega = v_const / R if R != float('inf') else 0

            theta += omega * dt
            x += v_const * math.cos(theta) * dt
            y += v_const * math.sin(theta) * dt

            res = calcular_parametros(v_const, steering, ax=0)

            # Integração da aceleração lateral para obter vy
            ay = res["ay"]
            vy += ay * dt

            tempos.append(t)
            x_list.append(x)
            y_list.append(y)
            vx_list.append(res["vx"])
            vy_list.append(vy)
            ax_list.append(res["ax"])
            ay_list.append(ay)
            yaw_list.append(res["Yaw Rate (deg/s)"])
            slip_list.append(res["Slip Angle (deg)"])
            wfi_list.append(res["wfi"])
            wfo_list.append(res["wfo"])
            wri_list.append(res["wri"])
            wro_list.append(res["wro"])

            t += dt

        # === Gráficos ===
        plt.figure(figsize=(14, 14))

        plt.subplot(6, 1, 1)
        plt.plot(tempos, vx_list, label="vx (m/s)")
        plt.plot(tempos, vy_list, label="vy (m/s)")
        plt.ylabel("Velocidade (m/s)")
        plt.title("Velocidades")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 2)
        plt.plot(tempos, ax_list, label="ax (m/s²)")
        plt.plot(tempos, ay_list, label="ay (m/s²)")
        plt.ylabel("Aceleração (m/s²)")
        plt.title("Acelerações")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 3)
        plt.plot(tempos, yaw_list, label="Yaw Rate (°/s)", color="green")
        plt.ylabel("Yaw Rate")
        plt.title("Velocidade Angular")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 4)
        plt.plot(tempos, slip_list, label="Slip Angle (°)", color="purple")
        plt.ylabel("Slip Angle")
        plt.title("Ângulo de Escorregamento")
        plt.grid(True)
        plt.legend()

        plt.subplot(6, 1, 5)
        plt.plot(tempos, wfi_list, label="wfi")
        plt.plot(tempos, wfo_list, label="wfo")
        plt.plot(tempos, wri_list, label="wri")
        plt.plot(tempos, wro_list, label="wro")
        plt.ylabel("Peso (N)")
        plt.xlabel("Tempo (s)")
        plt.title("Distribuição de Peso nas Rodas")
        plt.grid(True)
        plt.legend()

        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.suptitle("Simulação do Trajeto C", fontsize=14)
        plt.show()

        plt.figure(figsize=(8, 6))
        plt.plot(x_list, y_list, label='Trajetória XY', color='blue')
        plt.xlabel("Posição X (m)")
        plt.ylabel("Posição Y (m)")
        plt.title("Posição do Veículo (Trajetória XY)")
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()
        return

    # Se for trajeto A ou B, continua com o código anterior (não alterado)
    print("Para este código, apenas o Trajeto C foi corrigido com integração realista de vy.")

# ===== Execução principal =====
print("Escolha o trajeto:")
print("A - condução em linha reta até atingir 60km/h")
print("B - Condução em curva com um angulo de volante constante de -100º até atingir a velocidade de 60km/h.")
print("C - Ângulo do volante variando de -150 até 150 graus a uma velocidade de 20km/h")

escolha = input("Digite A, B ou C: ").strip().upper()
simular_trajeto(escolha)