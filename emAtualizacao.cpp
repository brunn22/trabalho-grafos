#include <iostream>
#include <vector>
#include <stack>
#include <queue>
#include <algorithm>  // Para std::sort

using namespace std;

// Função para realizar a busca em profundidade (DFS)
void dfs(int v, vector<vector<int>>& adj, vector<bool>& visitado) {
    stack<int> pilha;
    pilha.push(v);
    visitado[v] = true;

    while (!pilha.empty()) {
        int u = pilha.top();
        pilha.pop();

        for (int i : adj[u]) {
            if (!visitado[i]) {
                visitado[i] = true;
                pilha.push(i);
            }
        }
    }
}

// Função para realizar a busca em profundidade (DFS) e coletar os vértices de uma componente conexa
void dfsComponentes(int v, vector<vector<int>>& adj, vector<bool>& visitado, vector<int>& componente) {
    stack<int> pilha;
    pilha.push(v);
    visitado[v] = true;

    while (!pilha.empty()) {
        int u = pilha.top();
        pilha.pop();
        componente.push_back(u);

        for (int i : adj[u]) {
            if (!visitado[i]) {
                visitado[i] = true;
                pilha.push(i);
            }
        }
    }
}

// Função para verificar se o grafo é conexo
bool ehConexo(int n, vector<vector<int>>& adj) {
    vector<bool> visitado(n, false);

    // Começa a DFS a partir do vértice 0
    dfs(0, adj, visitado);

    // Verifica se todos os vértices foram visitados
    for (bool v : visitado) {
        if (!v) return false;  // Se algum vértice não foi visitado, o grafo não é conexo
    }

    return true;  // Todos os vértices foram visitados, o grafo é conexo
}

// Função para verificar se o grafo é bipartido
bool ehBipartido(int n, vector<vector<int>>& adj) {
    vector<int> cor(n, -1);  // -1 indica que o vértice ainda não foi colorido

    for (int i = 0; i < n; i++) {
        if (cor[i] == -1) {  // Se o vértice não foi colorido, inicie a BFS
            queue<int> fila;
            fila.push(i);
            cor[i] = 0;  // Inicia colorindo o vértice com a cor 0

            while (!fila.empty()) {
                int v = fila.front();
                fila.pop();

                for (int u : adj[v]) {
                    if (cor[u] == -1) {
                        // Se o vértice adjacente não foi colorido, colore com a cor oposta
                        cor[u] = 1 - cor[v];
                        fila.push(u);
                    } else if (cor[u] == cor[v]) {
                        // Se o vértice adjacente já foi colorido com a mesma cor, o grafo não é bipartido
                        return false;
                    }
                }
            }
        }
    }

    return true;  // O grafo é bipartido
}

// Função para verificar se o grafo é euleriano
bool ehEuleriano(int n, vector<vector<int>>& adj, string tipo_grafo) {
    if (tipo_grafo == "nao_direcionado") {
        // Para grafos não direcionados, todos os vértices devem ter grau par e o grafo deve ser conexo
        if (!ehConexo(n, adj)) return false;

        for (int i = 0; i < n; i++) {
            if (adj[i].size() % 2 != 0) return false;  // Verifica se o grau é ímpar
        }
        return true;  // Todos os vértices têm grau par, logo é euleriano
    } else if (tipo_grafo == "direcionado") {
        // Para grafos direcionados, todos os vértices devem ter o grau de entrada igual ao grau de saída
        vector<int> grauEntrada(n, 0), grauSaida(n, 0);

        for (int i = 0; i < n; i++) {
            for (int v : adj[i]) {
                grauSaida[i]++;
                grauEntrada[v]++;
            }
        }

        // Verifica se o grau de entrada é igual ao grau de saída para todos os vértices
        for (int i = 0; i < n; i++) {
            if (grauEntrada[i] != grauSaida[i]) return false;
        }

        // Verifica se o grafo é fortemente conexo
        vector<bool> visitado(n, false);
        dfs(0, adj, visitado);

        // Verifica se todos os vértices foram visitados
        for (bool v : visitado) {
            if (!v) return false;
        }

        // Agora, verificar a conectividade no grafo reverso
        vector<vector<int>> adjReverso(n);
        for (int i = 0; i < n; i++) {
            for (int v : adj[i]) {
                adjReverso[v].push_back(i);
            }
        }

        // Reseta o vetor de visitados e faz uma nova DFS no grafo reverso
        fill(visitado.begin(), visitado.end(), false);
        dfs(0, adjReverso, visitado);

        // Verifica se todos os vértices foram visitados no grafo reverso
        for (bool v : visitado) {
            if (!v) return false;
        }

        return true;  // Grau de entrada == grau de saída e o grafo é fortemente conexo
    }
    return false;
}

// Função para calcular e imprimir as componentes conexas em ordem lexicográfica
void componentesConexas(int n, vector<vector<int>>& adj) {
    vector<bool> visitado(n, false);

    vector<vector<int>> componentes;

    for (int i = 0; i < n; i++) {
        if (!visitado[i]) {
            vector<int> componente;
            dfsComponentes(i, adj, visitado, componente);
            // Ordena a componente em ordem crescente
            sort(componente.begin(), componente.end());
            componentes.push_back(componente);
        }
    }

    // Ordena as componentes pelo menor vértice em cada componente
    sort(componentes.begin(), componentes.end(), [](const vector<int>& a, const vector<int>& b) {
        return a.front() < b.front();
    });

    // Imprime as componentes
    for (const auto& c : componentes) {
        for (int v : c) {
            cout << v << " ";
        }
        cout << endl;
    }
}

// Função para verificar se o grafo tem ciclo
bool temCiclo(int n, vector<vector<int>>& adj, string tipo_grafo) {
    vector<bool> visitado(n, false);
    vector<bool> emRecursao(n, false);  // Usado para rastrear os vértices no caminho de recursão (para detectar ciclos em grafos direcionados)

    function<bool(int)> dfsCiclo = [&](int v) {
        visitado[v] = true;
        emRecursao[v] = true;

        for (int u : adj[v]) {
            if (!visitado[u]) {
                if (dfsCiclo(u)) return true;
            } else if (emRecursao[u]) {
                // Se o vértice já está no caminho de recursão, há um ciclo
                return true;
            }
        }

        emRecursao[v] = false;
        return false;
    };

    for (int i = 0; i < n; i++) {
        if (!visitado[i]) {
            if (dfsCiclo(i)) return true;
        }
    }

    return false;
}

// Função para calcular e imprimir as componentes fortemente conexas usando o Algoritmo de Kosaraju
void componentesFortementeConexas(int n, vector<vector<int>>& adj, const string& tipo_grafo) {
    if (tipo_grafo == "nao_direcionado") {
        cout << -1 << endl;  // Não disponível para grafos não direcionados
        return;
    }

    vector<vector<int>> adjReverso(n);
    for (int i = 0; i < n; i++) {
        for (int v : adj[i]) {
            adjReverso[v].push_back(i);
        }
    }

    vector<bool> visitado(n, false);
    stack<int> pilha;

    for (int i = 0; i < n; i++) {
        if (!visitado[i]) {
            dfs(i, adj, visitado);
            pilha.push(i);
        }
    }

    fill(visitado.begin(), visitado.end(), false);
    vector<vector<int>> componentes;

    while (!pilha.empty()) {
        int v = pilha.top();
        pilha.pop();

        if (!visitado[v]) {
            vector<int> componente;
            dfsComponentes(v, adjReverso, visitado, componente);
            // Ordena a componente em ordem crescente
            sort(componente.begin(), componente.end());
            componentes.push_back(componente);
        }
    }

    // Ordena as componentes pelo menor vértice em cada componente
    sort(componentes.begin(), componentes.end(), [](const vector<int>& a, const vector<int>& b) {
        return a.front() < b.front();
    });

    // Imprime as componentes
    for (const auto& c : componentes) {
        for (int v : c) {
            cout << v << " ";
        }
        cout << endl;
    }
}

// Função auxiliar para encontrar a trilha euleriana
void encontrarTrilhaEuleriana(int v, vector<vector<int>>& adj, stack<int>& resultado) {
    while (!adj[v].empty()) {
        int u = adj[v].back();
        adj[v].pop_back();
        encontrarTrilhaEuleriana(u, adj, resultado);
    }
    resultado.push(v);
}

// Função para calcular e imprimir uma trilha euleriana
void trilhaEuleriana(int n, vector<vector<int>>& adj, const string& tipo_grafo) {
    if (tipo_grafo == "nao_direcionado") {
        // Verifica se o grafo é euleriano e pode ter uma trilha euleriana
        if (!ehConexo(n, adj)) {
            cout << -1 << endl;  // Não disponível para grafos desconexos
            return;
        }

        int oddCount = 0;
        for (int i = 0; i < n; i++) {
            if (adj[i].size() % 2 != 0) {
                oddCount++;
            }
        }

        if (oddCount != 0 && oddCount != 2) {
            cout << -1 << endl;  // Não pode ter uma trilha euleriana
            return;
        }

        // Copia o grafo para manipulação
        vector<vector<int>> adjCopia = adj;
        stack<int> resultado;

        // Encontrar o primeiro vértice com arestas (para garantir ordem lexicográfica)
        int inicio = 0;
        for (int i = 0; i < n; i++) {
            if (!adjCopia[i].empty()) {
                inicio = i;
                break;
            }
        }

        encontrarTrilhaEuleriana(inicio, adjCopia, resultado);

        // Imprime o resultado
        vector<int> caminho;
        while (!resultado.empty()) {
            caminho.push_back(resultado.top());
            resultado.pop();
        }

        // Imprime o caminho em ordem correta
        for (int i = caminho.size() - 1; i >= 0; i--) {
            cout << caminho[i] << " ";
        }
        cout << endl;

    } else if (tipo_grafo == "direcionado") {
        // Verifica se o grafo é euleriano e pode ter uma trilha euleriana
        vector<int> grauEntrada(n, 0), grauSaida(n, 0);

        for (int i = 0; i < n; i++) {
            for (int u : adj[i]) {
                grauSaida[i]++;
                grauEntrada[u]++;
            }
        }

        int start = -1, end = -1;
        for (int i = 0; i < n; i++) {
            if (grauSaida[i] - grauEntrada[i] == 1) {
                if (start != -1) {
                    cout << -1 << endl;  // Mais de um vértice com grau de saída - grau de entrada = 1
                    return;
                }
                start = i;
            } else if (grauEntrada[i] - grauSaida[i] == 1) {
                if (end != -1) {
                    cout << -1 << endl;  // Mais de um vértice com grau de entrada - grau de saída = 1
                    return;
                }
                end = i;
            } else if (grauEntrada[i] != grauSaida[i]) {
                cout << -1 << endl;  // Vértice com grau de entrada diferente do grau de saída
                return;
            }
        }

        if (start == -1) {
            start = 0;  // Se não houver vértice com grau de saída - grau de entrada = 1, começa em qualquer vértice
        }

        vector<vector<int>> adjCopia = adj;
        stack<int> resultado;

        encontrarTrilhaEuleriana(start, adjCopia, resultado);

        // Imprime o resultado
        vector<int> caminho;
        while (!resultado.empty()) {
            caminho.push_back(resultado.top());
            resultado.pop();
        }

        // Imprime o caminho em ordem correta
        for (int i = caminho.size() - 1; i >= 0; i--) {
            cout << caminho[i] << " ";
        }
        cout << endl;
    }
}

void imprimirVerticesArticulacao(int n, vector<vector<int>>& adj) {
    vector<bool> visitado(n, false);
    vector<int> low(n, -1);
    vector<int> disc(n, -1);
    vector<int> pais(n, -1);
    vector<bool> articulacao(n, false);
    int tempo = 0;

    // Função auxiliar para encontrar vértices de articulação
    function<void(int)> encontrarVerticesArticulacao = [&](int v) {
        int filhos = 0;
        visitado[v] = true;
        disc[v] = low[v] = ++tempo;

        for (int u : adj[v]) {
            if (!visitado[u]) {
                pais[u] = v;
                filhos++;
                encontrarVerticesArticulacao(u);

                low[v] = min(low[v], low[u]);

                if (pais[v] == -1 && filhos > 1) {
                    articulacao[v] = true;
                }
                if (pais[v] != -1 && low[u] >= disc[v]) {
                    articulacao[v] = true;
                }
            } else if (u != pais[v]) {
                low[v] = min(low[v], disc[u]);
            }
        }
    };

    // Encontrar vértices de articulação para cada componente conexo
    for (int i = 0; i < n; i++) {
        if (!visitado[i]) {
            encontrarVerticesArticulacao(i);
        }
    }

    // Imprimir os vértices de articulação em ordem lexicográfica
    vector<int> verticesArticulacao;
    for (int i = 0; i < n; i++) {
        if (articulacao[i]) {
            verticesArticulacao.push_back(i);
        }
    }

    sort(verticesArticulacao.begin(), verticesArticulacao.end());

    for (int v : verticesArticulacao) {
        cout << v << " ";
    }
    cout << endl;
}

// Função auxiliar para encontrar arestas ponte
void encontrarArestasPonte(int v, int &tempo, vector<vector<int>>& adj, vector<int>& low, vector<int>& disc, vector<int>& pais, vector<bool>& visitado, vector<pair<int, int>>& arestasPonte) {
    visitado[v] = true;
    disc[v] = low[v] = ++tempo;

    for (int u : adj[v]) {
        if (!visitado[u]) {
            pais[u] = v;
            encontrarArestasPonte(u, tempo, adj, low, disc, pais, visitado, arestasPonte);

            low[v] = min(low[v], low[u]);

            if (low[u] > disc[v]) {
                arestasPonte.push_back({v, u});
            }
        } else if (u != pais[v]) {
            low[v] = min(low[v], disc[u]);
        }
    }
}

// Função para calcular e imprimir se há arestas ponte
void calcularArestasPonte(int n, vector<vector<int>>& adj) {
    vector<int> low(n, -1);
    vector<int> disc(n, -1);
    vector<int> pais(n, -1);
    vector<bool> visitado(n, false);
    vector<pair<int, int>> arestasPonte;
    int tempo = 0;

    for (int i = 0; i < n; i++) {
        if (!visitado[i]) {
            encontrarArestasPonte(i, tempo, adj, low, disc, pais, visitado, arestasPonte);
        }
    }

    // Verifica se há alguma aresta ponte e imprime o resultado
    if (!arestasPonte.empty()) {
        cout << 1 << endl;
    } else {
        cout << 0 << endl;
    }
}

void arvoreProfundidade(int n, const vector<vector<int>>& adj) {
    vector<bool> visited(n, false);
    vector<int> ordem;

    for (int i = 0; i < n; i++) {
        if (!visited[i]) {
            stack<int> s;
            s.push(i);
            visited[i] = true;

            while (!s.empty()) {
                int u = s.top();
                s.pop();
                ordem.push_back(u);

                for (int v : adj[u]) {
                    if (!visited[v]) {
                        s.push(v);
                        visited[v] = true;
                    }
                }
            }
        }
    }

    // Imprime a ordem dos vértices na árvore de profundidade
    for (size_t i = 0; i < ordem.size(); i++) {
        if (i > 0) cout << " ";
        cout << ordem[i];
    }
    cout << endl;
}

void arvoreLargura(int n, const vector<vector<int>>& adj) {
    vector<bool> visited(n, false);
    vector<int> ordem;
    queue<int> q;
    
    visited[0] = true;  // Começa pela raiz 0
    q.push(0);

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        ordem.push_back(u);

        // Adiciona os vizinhos não visitados à fila, priorizando a ordem lexicográfica
        vector<int> vizinhos = adj[u];
        sort(vizinhos.begin(), vizinhos.end());  // Ordena os vizinhos
        for (int v : vizinhos) {
            if (!visited[v]) {
                visited[v] = true;
                q.push(v);
            }
        }
    }

    // Imprime a ordem dos vértices visitados
    for (int v : ordem) {
        cout << v << " ";
    }
    cout << endl;
}

int main() {
    int n, m;
    cin >> n >> m;

    string tipo_grafo;
    cin >> tipo_grafo;

    vector<vector<int>> adj(n);

    for (int i = 0; i < m; i++) {
        int id, u, v, peso;
        cin >> id >> u >> v >> peso;
        adj[u].push_back(v);
        if (tipo_grafo == "nao_direcionado") {
            adj[v].push_back(u);
        }
    }

    if (ehConexo(n, adj)) {
        cout << 1 << endl;  // O grafo é conexo
    } else {
        cout << 0 << endl;  // O grafo não é conexo
    }

    if (ehBipartido(n, adj)) {
        cout << 1 << endl;  // O grafo é bipartido
    } else {
        cout << 0 << endl;  // O grafo não é bipartido
    }

    if (ehEuleriano(n, adj, tipo_grafo)) {
        cout << 1 << endl;  // O grafo é euleriano
    } else {
        cout << 0 << endl;  // O grafo não é euleriano
    }

    if (temCiclo(n, adj, tipo_grafo)) {
        cout << 1 << endl;  // O grafo tem ciclo
    } else {
        cout << 0 << endl;  // O grafo não tem ciclo
    }

    // Chama a função para calcular e imprimir as componentes conexas
    componentesConexas(n, adj);

    // Chama a função para calcular e imprimir as componentes fortemente conexas
    componentesFortementeConexas(n, adj, tipo_grafo);
    
    // Chama a função para calcular e imprimir uma trilha euleriana
    trilhaEuleriana(n, adj, tipo_grafo);
    
    // Chama a função para imprimir os vértices de articulação
    imprimirVerticesArticulacao(n, adj);
    
    // Chama a função para calcular e imprimir a quantidade de arestas ponte
    calcularArestasPonte(n, adj);
    
    //Chama a função para calcular e imprimir a árvore de profundidade
    arvoreProfundidade(n, adj);
    
    //Chama a função para calcular e imprimir a árvore de largura
    arvoreLargura(n, adj);

    return 0;
} 