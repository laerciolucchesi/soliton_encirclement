# Equações do controle tangencial (domínio do tempo)

Este arquivo reúne as equações do controle tangencial na forma contínua (notação \(\dot x = dx/dt\)) e no formato mais próximo do que está implementado.

> Observação: para ver as equações renderizadas, abra o Preview do Markdown (`Ctrl+Shift+V`). Para renderizar matemática, use uma extensão como **Markdown Preview Enhanced** ou **Markdown+Math**.

---

## 1) Geometria em torno do alvo (plano XY)

Considere o alvo com posição \(p_T(t)\in\mathbb{R}^2\) e o agente \(i\) com posição \(p_i(t)\in\mathbb{R}^2\).

$$
\mathbf r_i(t) = p_i(t) - p_T(t),
\qquad
r_i(t) = \|\mathbf r_i(t)\|.
$$

$$
\hat{\mathbf r}_i(t) = \frac{\mathbf r_i(t)}{r_i(t)}.
$$

A direção tangencial unitária (sentido anti-horário) é

$$
\hat{\mathbf t}_i(t) = \begin{bmatrix}-\hat r_{iy}(t)\\ \hat r_{ix}(t)\end{bmatrix}.
$$

Para evitar singularidade quando \(r_i\to 0\), define-se tipicamente

$$
r_{\mathrm{eff},i}(t) = \max\{r_i(t),\,R_{\min}\}.
$$

---

## 2) Erro de espaçamento tangencial (adimensional)

Sejam \(\Delta\theta_{i^-}(t)\) o gap angular até o predecessor \(i^-\) e \(\Delta\theta_{i^+}(t)\) o gap até o sucessor \(i^+\). Com pesos (lambdas) \(\lambda_{i^-}>0\) e \(\lambda_{i^+}>0\), o erro de espaçamento é

$$
e_{\tau,i}(t)=
\frac{\lambda_{i^-}\,\Delta\theta_{i^+}(t)-\lambda_{i^+}\,\Delta\theta_{i^-}(t)}
{\lambda_{i^-}\,\Delta\theta_{i^+}(t)+\lambda_{i^+}\,\Delta\theta_{i^-}(t)}.
$$

### Espaçamento arbitrário (não-uniforme) via \(\lambda\)

Essa forma ponderada permite impor **espaçamento arbitrário** (isto é, gaps não-uniformes) sem usar o número global de agentes \(N\). A condição de equilíbrio local \(e_{\tau,i}=0\) equivale a

$$
\lambda_{i^-}\,\Delta\theta_{i^+} = \lambda_{i^+}\,\Delta\theta_{i^-}.
$$

Portanto, ao aumentar \(\lambda\) de um arco, o controlador tende a aumentar o gap do arco oposto para reequilibrar a condição acima (na prática, isso “puxa” a formação para produzir gaps desejados).

**Como isso é implementado no código:**

- O alvo pode transmitir um mapa \(\{\lambda_j\}\) em `TargetState.alive_lambdas`.
- Convenção usada: \(\lambda_j\) está associado ao arco \((j \to succ(j))\).
- Para o agente \(i\):
	- \(\lambda_{i^-}\) (no código: `lp_pred`) é lido usando o ID do predecessor.
	- \(\lambda_{i^+}\) (no código: `lp_succ`) é lido usando o próprio ID do agente.
- Se uma chave não existir (ou estiver inválida), o valor padrão é 1.0; e se o alvo parar de transmitir, o agente mantém o último valor válido (regra de persistência).

O caso de espaçamento uniforme é recuperado com \(\lambda_j=1\,\forall j\), que reduz \(e_{\tau,i}\) à razão de contraste não-ponderada.

---

## 3) Amortecimento opcional por velocidade angular (local)

A velocidade angular do agente em torno do alvo, no plano XY, pode ser estimada pela componente tangencial da velocidade relativa:

$$
\omega_i(t) = \frac{\big(\dot p_i(t) - \dot p_T(t)\big)\cdot \hat{\mathbf t}_i(t)}{r_i(t)}.
$$

(na prática usa-se \(r_{\mathrm{eff}}\) para robustez numérica).

Define-se uma referência local baseada nos vizinhos, por exemplo a média quando ambos existem:

$$
\omega^{\mathrm{local}}_{\mathrm{ref},i}(t)=\frac{1}{2}\big(\omega_{i^-}(t)+\omega_{i^+}(t)\big).
$$

O erro de espaçamento efetivo, com amortecimento opcional, é

$$
e^{\mathrm{eff}}_{\tau,i}(t)
= e_{\tau,i}(t) - K_{\Omega}\big(\omega_i(t)-\omega^{\mathrm{local}}_{\mathrm{ref},i}(t)\big).
$$

---

## 4) Dinâmica do estado tangencial (soliton) \(u_i(t)\)

A dinâmica contínua do estado interno \(u_i(t)\) (um escalar por agente) é:

$$
\boxed{
\dot u_i(t)=
C\big(u_{i^+}(t)-u_{i^-}(t)\big)
-\beta\,u_i(t)
-\alpha\,u_i^3(t)
+K_{E_\tau}\,e^{\mathrm{eff}}_{\tau,i}(t)
+\kappa\big(u_{i^+}(t)-2u_i(t)+u_{i^-}(t)\big)
}
$$

onde:

- \(C\): acoplamento antissimétrico (convecção no índice do anel)
- \(\beta\): amortecimento linear
- \(\alpha\): não-linearidade cúbica
- \(K_{E_\tau}\): ganho de injeção do erro de espaçamento
- \(K_{\Omega}\): ganho do amortecimento por \(\omega\) (opcional)
- \(\kappa\): ganho de difusão/Laplaciano discreto (opcional)

### Escala espacial do Laplaciano (\(\Delta s\))

O termo

$$
\kappa\big(u_{i^+}(t)-2u_i(t)+u_{i^-}(t)\big)
$$

é o **Laplaciano discreto** no índice do anel (segunda derivada “espacial” ao longo do anel). No contínuo, se você modela o estado como \(u=u(s,t)\) ao longo de uma coordenada \(s\) (comprimento de arco), o termo de difusão seria

$$
\kappa_s\,\frac{\partial^2 u}{\partial s^2}(s,t).
$$

Discretizando \(s\) em amostras \(s_i=i\,\Delta s\), tem-se a aproximação padrão

$$
\frac{\partial^2 u}{\partial s^2}(s_i,t) \approx \frac{u_{i+1}(t)-2u_i(t)+u_{i-1}(t)}{(\Delta s)^2}.
$$

Logo, para compatibilizar “contínuo vs discreto”:

$$
\dot u_i(t)=\cdots + \underbrace{\frac{\kappa_s}{(\Delta s)^2}}_{\kappa_{\text{discreto}}}\big(u_{i+1}(t)-2u_i(t)+u_{i-1}(t)\big).
$$

No código, como o termo é implementado como `+ KAPPA_U_DIFF * (u_succ - 2u + u_pred)` **sem dividir por \((\Delta s)^2\)**, o parâmetro `KAPPA_U_DIFF` corresponde a

$$
\boxed{\kappa_{\text{código}} = \kappa_{\text{discreto}} = \frac{\kappa_s}{(\Delta s)^2}}
\qquad\Longleftrightarrow\qquad
\boxed{\kappa_s = \kappa_{\text{código}}\,(\Delta s)^2}.
$$

Se você quiser uma escolha geométrica para \(\Delta s\): em um anel de raio aproximadamente \(R\) com \(N\) agentes uniformemente distribuídos,

$$
\Delta\theta \approx \frac{2\pi}{N},
\qquad
\Delta s \approx R\,\Delta\theta \approx \frac{2\pi R}{N}.
$$

Em geral, pode-se tomar \(R\approx\) `ENCIRCLEMENT_RADIUS` como escala. Se o espaçamento não for uniforme (falhas, \(\lambda\) variáveis), então \(\Delta s\) deixa de ser constante; nesse caso, o Laplaciano por índice continua sendo um “suavizador” útil, mas a interpretação física via \(\Delta s\) é apenas aproximada.

Se você expandir o termo de \(e^{\mathrm{eff}}_{\tau,i}\), aparece explicitamente a contribuição de amortecimento:

$$
K_{E_\tau}\,e^{\mathrm{eff}}_{\tau,i}
=K_{E_\tau}\,e_{\tau,i}-K_{E_\tau}K_{\Omega}\big(\omega_i-\omega^{\mathrm{local}}_{\mathrm{ref},i}\big).
$$

---

## 5) Mapeamento de \(u_i(t)\) para velocidade tangencial comandada

A velocidade tangencial comandada no plano XY é proporcional a \(u_i\):

$$
\boxed{
\mathbf v_{\tau,i}(t)=\big(K_{\tau}\,u_i(t)\,r_{\mathrm{eff},i}(t)\big)\,\hat{\mathbf t}_i(t)
}
$$

Com esse mapeamento, quando \(r_i\) não é muito pequeno, a velocidade angular induzida tende a

$$
\omega_i(t) \approx \frac{\mathbf v_{\tau,i}(t)\cdot \hat{\mathbf t}_i(t)}{r_i(t)} \approx K_{\tau}\,u_i(t).
$$

---

## 6) (Opcional) Spin global comandado pelo alvo

Se o alvo difunde uma referência \(\omega^{\mathrm{target}}_{\mathrm{ref}}(t)\) comum (spin global), um termo adicional pode ser somado à velocidade tangencial:

$$
\mathbf v_{\mathrm{spin},i}(t)=\big(\omega^{\mathrm{target}}_{\mathrm{ref}}(t)\,r_i(t)\big)\,\hat{\mathbf t}_i(t).
$$

Esse termo é independente da dinâmica \(u_i\) e é simplesmente somado ao comando final.
