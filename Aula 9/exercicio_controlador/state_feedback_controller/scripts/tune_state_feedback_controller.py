# Script auxiliar para sintonizar um controlador de realimentação de estados
# Aqui fazemos isso usando o conceito de um controlador LQR (Linear Quadratic Regulator)
#  para determinar os ganhos do feedback de posição e velocidade.
# Para mais detalhes, seguem alguns recursos sugeridos:
#
# https://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=ControlStateSpace#6
# https://underactuated.mit.edu/lqr.html

import control as ct
import numpy as np

# Seja x a posição do carrinho, v sua velocidade, a sua aceleração
# A dinâmica de um carrinho de massa m acelerado por uma força u é dada
#   pela seguinte equação diferencial (que é a segunda lei de newton):
#     m * a = u
# Mas também, sabemos que dx/dt = v, e d²x/dt² = dv/dt = a
# Podemos escrever isso de forma matricial, como:
#
#    [ dx/dt = v ] = [ 0 , 1 ] * [ x ] + [  0  ] * u 
#    [ dv/dt = a ] = [ 0 , 0 ]   [ v ]   [ 1/m ]
#    
# Ou também como dz/dt = A*z + B*u, onde z = [x; v].
#  Declaramos a massa do carrinho e essas matrizes A e B abaixo:

m = 1.0 # [kg]

A = np.array([[0, 1],[0, 0]])
B = np.array([[0],[1/m]])

# Denotemos agora um estado de referência zr = [xr; vr], com a posição e 
#   velocidade desejada do carrinho, que é atingido com uma força de referência ur.
# Denotemos o erro de seguimento de estados como e = z - zr; logo z = e + zr
# A derivada do erro é de/dt = dz/dt - dzr/dt = A*z + B*u - (A*zr + B*ur)
# Agrupando termos e substituindo e = z - zr, w = u - ur, temos a seguinte dinâmica do erro:
#  de/dt = A*(z - zr) + B*(u - ur) = A*e + B*w
#
# Vamos agora aplicar a entrada u como o termo de referência ur somado a uma realimentação do erro:
#   u = ur - K*e
#  onde K é a matriz de ganhos de realimentação que queremos sintonizar.
# Nesse caso, como queremos posicionar o carrinho em uma posição desejada xr, com vr = 0, 
#  temos que em regime permanente ur = 0; logo, só temos que aplicar u = -K*e.

# Para sintonizar K, usamos o método LQR para minimizar um custo J = ∫(e^T*Q*e + w^T*R*w) dt
# Aqui, Q é uma matriz de ponderação dos erros de estado, e R é uma matriz de ponderação
#  dos esforços aplicados.
# Digamos que queremos um custo 10 vezes maior para erros de posição que para erros de velocidade v,

Q = np.array([[10, 0], [0, 1]])

# ... e que o custo de aplicar uma força é bem baixo, digamos R = 0.5

R = np.array([[0.5]])

# A função lqr do pacote control calcula a matriz de ganhos K ótima para nós:

K = ct.lqr(A,B,Q,R)[0]

print("Matriz de ganhos K do controlador LQR: ")
print(K)

# Para garantir que o nosso sistema em malha fechada tem a dinâmica desejada,
#  podemos checar os autovalores da matriz (A - B*K).
# Se todos eles tiverem parte real negativa, então ela é estável.

res = np.linalg.eig(A-B*K)
print("Autovalores do sistema em malha fechada: ")
print(res[0])

# ... e feito! Agora é só colocar essa matriz K no arquivo de configuração YAML.

