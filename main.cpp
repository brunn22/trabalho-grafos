#include <iostream>
#include <stack>
#include <vector>
#include <string>
#include <sstream>
#include <climits>
#include <set>
#include <queue>
#include <algorithm>
#include <limits>
#include <functional>

using namespace std;

// Função de busca em profundidade (DFS)
void DFS(int v, vector<bool>& visited, const vector<vector<int>>& adj) {
    visited[v] = true; // Marca o vértice como visitado
    for (int u : adj[v]) { // Para cada vértice adjacente
        if (!visited[u]) { // Se não foi visitado
            DFS(u, visited, adj); // Chama a DFS recursivamente
        }
    }
}

// Função recursiva para realizar a DFS e registrar a árvore de profundidade
void dfs(int u, vector<bool>& visited, vector<vector<pair<int, int>>>& adj, vector<int>& arvoreProfundidade) {
    visited[u] = true; // Marca o vértice atual como visitado
    sort(adj[u].begin(), adj[u].end()); // Ordena as arestas adjacentes
    for (const auto& p : adj[u]) { // Itera por cada aresta adjacente
        int v = p.first; // Obtém o vértice adjacente
        int id = p.second; // Obtém o ID da aresta
        if (!visited[v]) { // Se não foi visitado
            arvoreProfundidade.push_back(id); // Adiciona o ID da aresta à árvore
            dfs(v, visited, adj, arvoreProfundidade); // Chama DFS para o vértice adjacente
        }
    }
}

// Função para detectar ciclos em um grafo dirigido
bool DFS_Ciclo(int v, vector<bool>& visited, vector<bool>& recStack, const vector<vector<int>>& adj) {
    visited[v] = true; // Marca o vértice como visitado
    recStack[v] = true; // Adiciona à pilha de recursão

    for (int u : adj[v]) { // Para cada vértice adjacente
        if (!visited[u] && DFS_Ciclo(u, visited, recStack, adj)) { // Se não visitado e ciclo encontrado
            return true;
        } else if (recStack[u]) { // Se o vértice está na pilha de recursão
            return true; // Retorna que há um ciclo
        }
    }

    recStack[v] = false; // Remove o vértice da pilha de recursão
    return false; // Não encontrou um ciclo
}

// Função utilitária para encontrar vértices de articulação
void encontrarVerticesArticulacaoUtil(int u, vector<bool>& visited, vector<int>& disc, vector<int>& low, vector<int>& parent, vector<bool>& articulationPoints, vector<vector<int>>& adj) {
    static int time = 0; // Tempo de descoberta
    int children = 0; // Contador de filhos
    visited[u] = true; // Marca o vértice como visitado
    disc[u] = low[u] = ++time; // Define os tempos de descoberta e baixo

    for (int v : adj[u]) { // Para cada vértice adjacente
        if (!visited[v]) { // Se não foi visitado
            children++; // Aumenta o contador de filhos
            parent[v] = u; // Define o pai
            encontrarVerticesArticulacaoUtil(v, visited, disc, low, parent, articulationPoints, adj); // Chama recursivamente
            low[u] = min(low[u], low[v]); // Atualiza o valor de low

            // Condição para identificar um vértice de articulação
            if (parent[u] == -1 && children > 1) {
                articulationPoints[u] = true; // Articulação se raiz com mais de um filho
            }
            if (parent[u] != -1 && low[v] >= disc[u]) {
                articulationPoints[u] = true; // Articulação se um filho tem low maior ou igual ao disc
            }
        } else if (v != parent[u]) { // Se o vértice é adjacente e não é o pai
            low[u] = min(low[u], disc[v]); // Atualiza low
        }
    }
}

// Estrutura para representar uma aresta
struct Aresta {
    int id; // ID da aresta
    int origem; // Vértice de origem
    int destino; // Vértice de destino
    int peso; // Peso da aresta
};

// Classe que representa um grafo
class Grafo {
public:
    int numVertices; // Número de vértices
    int numArestas; // Número de arestas
    bool direcionado; // Indica se é um grafo direcionado
    vector<Aresta> arestas; // Lista de arestas
    vector<vector<int>> adj; // Lista de adjacência

    // Construtor
    Grafo(int v, int a, bool dir) : numVertices(v), numArestas(a), direcionado(dir) {
        adj.resize(numVertices); // Inicializa a lista de adjacência
    }

    // Função para adicionar uma aresta
    void adicionarAresta(int id, int origem, int destino, int peso) {
        Aresta aresta = {id, origem, destino, peso}; // Cria uma nova aresta
        arestas.push_back(aresta); // Adiciona à lista de arestas
        adj[aresta.origem].push_back(aresta.destino); // Adiciona o destino à lista de adjacência
        if (!direcionado) {
            adj[aresta.destino].push_back(aresta.origem); // Adiciona a origem se não for direcionado
        }
    }

    // Função para verificar a conectividade do grafo
    void conexo() {
        vector<bool> visited(numVertices, false); // Vetor de visitados
        if (direcionado) {
            vector<vector<int>> undirectedAdj = adj; // Copia a lista de adjacência
            for (int u = 0; u < numVertices; ++u) {
                for (int v : adj[u]) { // Para cada aresta, adiciona a adjacência inversa
                    undirectedAdj[v].push_back(u);
                }
            }
            DFS(0, visited, undirectedAdj); // Chama DFS na versão não direcionada
        } else {
            DFS(0, visited, adj); // Chama DFS no grafo direcionado
        }

        // Verifica se todos os vértices foram visitados
        for (int i = 0; i < numVertices; ++i) {
            if (!visited[i]) {
                cout << "0"; // Se não visitado, imprime 0
                return;
            }
        }
        cout << "1"; // Se todos visitados, imprime 1
    }

    // Função para verificar se o grafo é bipartido
    void bipartido() {
        cout << endl; // Linha nova para o output
        vector<int> cor(numVertices, -1); // Vetor para armazenar as cores dos vértices
        bool ehBipartido = true; // Flag para verificar bipartição

        for (int i = 0; i < numVertices; ++i) {
            if (cor[i] == -1) { // Se o vértice não foi colorido
                if (!bipartidoDFS(i, cor)) { // Chama DFS para bipartição
                    ehBipartido = false; // Se não é bipartido
                    break; // Sai do loop
                }
            }
        }

        cout << (ehBipartido ? "1" : "0"); // Imprime 1 se bipartido, 0 caso contrário
    }

    // Função recursiva para auxiliar na verificação da bipartição
    bool bipartidoDFS(int v, vector<int>& cor) {
        cor[v] = 0; // Atribui a primeira cor

        for (int u : adj[v]) { // Para cada adjacente
            if (cor[u] == -1) { // Se não colorido
                cor[u] = 1 - cor[v]; // Atribui a cor oposta
                if (!bipartidoDFS(u, cor)) return false; // Chama recursivamente
            } else if (cor[u] == cor[v]) { // Se a cor é igual a do adjacente
                return false; // Não é bipartido
            }
        }
        return true; // É bipartido
    }

    // Função para verificar se o grafo contém um circuito euleriano
    void euleriano() {
        cout << endl; // Linha nova para o output     
        vector<bool> visited(numVertices, false); // Vetor de vértices visitados
        DFS(0, visited, adj); // Chama DFS a partir do vértice 0

        // Verifica se todos os vértices têm grau par e foram visitados
        for (int i = 0; i < numVertices; ++i) {
            if (!visited[i]) {
                cout << "0"; // Se não visitado, imprime 0
                return;
            }
            if (adj[i].size() % 2 != 0) { // Se o grau é ímpar
                cout << "0"; // Se não for par, imprime 0
                return;
            }
        }
        cout << "1"; // Se contém caminho euleriano, imprime 1
    }

    // Função para verificar se há um ciclo no grafo
    void ciclo() {
        cout << endl; // Linha nova para o output
        vector<bool> visited(numVertices, false); // Vetor de vértices visitados
        vector<bool> recStack(numVertices, false); // Pilha de recursão

        // Para cada vértice, verifica se não foi visitado
        for (int i = 0; i < numVertices; ++i) {
            if (!visited[i] && DFS_Ciclo(i, visited, recStack, adj)) {
                cout << '1'; // Se um ciclo é encontrado, imprime 1
                return;
            }
        }
        
        cout << '0'; // Se nenhum ciclo é encontrado, imprime 0
    }

    // Função para contar componentes conexas
    void componentesConexas() {
        cout << endl; // Linha nova para o output
        
        if(direcionado) {
            cout << "-1"; // Não é aplicável a grafos direcionados
            return;
        } 
        vector<bool> visitado(numVertices, false); // Vetor de vértices visitados
        int componentes = 0; // Contador de componentes

        // Para cada vértice, verifica se não foi visitado
        for (int i = 0; i < numVertices; ++i) {
            if (!visitado[i]) {
                DFS(i, visitado, adj); // Chama DFS para o componente
                componentes++; // Incrementa o contador de componentes
            }
        }

        cout << componentes; // Imprime o número de componentes
    }

    // Função para encontrar componentes fortemente conexas usando Tarjan
    void tarjanDFS(int u, vector<int>& disc, vector<int>& low, vector<bool>& inStack, stack<int>& Stack, vector<vector<int>>& sccs) {
        static int time = 0; // Tempo de descoberta
        disc[u] = low[u] = ++time; // Atribui tempos de descoberta e baixo
        Stack.push(u); // Adiciona o vértice à pilha
        inStack[u] = true; // Marca como em pilha

        for (int v : adj[u]) { // Para cada adjacente
            if (disc[v] == -1) { // Se não visitado
                tarjanDFS(v, disc, low, inStack, Stack, sccs); // Chama recursivamente
                low[u] = min(low[u], low[v]); // Atualiza o valor de low
            } else if (inStack[v]) { // Se está na pilha
                low[u] = min(low[u], disc[v]); // Atualiza low
            }
        }

        // Se o vértice atual é raiz de um SCC
        if (low[u] == disc[u]) {
            vector<int> component; // Inicializa uma componente
            while (Stack.top() != u) { // Enquanto não chega ao vértice raiz
                int v = Stack.top(); // Obtém o vértice do topo
                Stack.pop(); // Remove da pilha
                inStack[v] = false; // Marca como não em pilha
                component.push_back(v); // Adiciona na componente atual
            }
            Stack.pop(); // Remove o vértice raiz
            inStack[u] = false; // Marca como não em pilha
            component.push_back(u); // Adiciona o vértice raiz na componente
            sccs.push_back(component); // Adiciona a componente às componentes fortemente conexas
        }
    }

    // Função para contar componentes fortemente conexas
    void componentesFortementeConexas() {
        cout << endl; // Linha nova para o output
        if(direcionado) {
            vector<int> disc(numVertices, -1); // Tempos de descoberta
            vector<int> low(numVertices, -1); // Valores de baixo
            vector<bool> inStack(numVertices, false); // Pilha de verificação
            stack<int> Stack; // Pilha de Tarjan
            vector<vector<int>> sccs; // Lista de componentes fortemente conexas

            for (int i = 0; i < numVertices; ++i) {
                if (disc[i] == -1) {
                    tarjanDFS(i, disc, low, inStack, Stack, sccs); // Chama função de Tarjan
                }
            }

            cout << sccs.size(); // Imprime o número de SCCs encontradas
        } else {
            cout << "-1"; // Não aplicável a grafos não direcionados
        }
    }

    // Função para encontrar vértices de articulação
    void verticesArticulacao() {
        cout << endl; // Linha nova para o output
        
        if (!direcionado) { // Se o grafo não é direcionado
            for (const auto& aresta : arestas) {
                adj[aresta.origem].push_back(aresta.destino); // Adiciona aresta de origem para destino
                adj[aresta.destino].push_back(aresta.origem); // Adiciona aresta de destino para origem
            }

            vector<bool> visited(numVertices, false); // Vetor de visitados
            vector<int> disc(numVertices, -1); // Vetor de descoberta
            vector<int> low(numVertices, -1); // Vetor de baixo
            vector<int> parent(numVertices, -1); // Vetor de pais
            vector<bool> articulationPoints(numVertices, false); // Vetor para pontos de articulação

            // Para cada vértice
            for (int i = 0; i < numVertices; i++) {
                if (!visited[i]) {
                    encontrarVerticesArticulacaoUtil(i, visited, disc, low, parent, articulationPoints, adj); // Chama utilitária
                }
            }

            bool hasArticulationPoint = false; // Flag para verificar ponto de articulação
            // Imprime todos os pontos de articulação encontrados
            for (int i = 0; i < numVertices; i++) {
                if (articulationPoints[i]) {
                    cout << i << " "; // Imprime o índice do ponto de articulação
                    hasArticulationPoint = true; // Marca que há ponto de articulação
                }
            }

            if (!hasArticulationPoint) {
                cout << "0"; // Se não há pontos de articulação, imprime 0
            }

        } else {
            cout << "-1"; // Não aplicável a grafos direcionados
        }
    }

    // Função para encontrar arestas de ponte
    void dfsTarjan(int u, vector<int>& disc, vector<int>& low, vector<int>& parent, vector<pair<int, int>>& pontes, int& time) {
        disc[u] = low[u] = ++time; // Atribui tempos de descoberta e baixo

        for (int v : adj[u]) { // Para cada vértice adjacente
            if (disc[v] == -1) { // Se não visitado
                parent[v] = u; // Define o pai
                dfsTarjan(v, disc, low, parent, pontes, time); // Chama recursivamente

                low[u] = min(low[u], low[v]); // Atualiza o valor de low

                if (low[v] > disc[u]) {
                    pontes.push_back({u, v}); // Adiciona a aresta à lista de pontes
                }
            } else if (v != parent[u]) { // Se não é o pai
                low[u] = min(low[u], disc[v]); // Atualiza low
            }
        }
    }

    // Função para contar arestas de ponte
    void arestasPonte() {
        cout << endl; // Linha nova para o output

        if (!direcionado) { // Se o grafo não é direcionado
            vector<int> disc(numVertices, -1); // Tempos de descoberta
            vector<int> low(numVertices, -1); // Valores de baixo
            vector<int> parent(numVertices, -1); // Vetor de pais
            vector<pair<int, int>> pontes; // Lista de pontes
            int time = 0; // Contador de tempo

            // Para cada vértice
            for (int i = 0; i < numVertices; i++) {
                if (disc[i] == -1) {
                    dfsTarjan(i, disc, low, parent, pontes, time); // Chama a função de Tarjan
                }
            }
            cout << pontes.size(); // Imprime o número de arestas de ponte
        } else {
            cout << "-1"; // Não aplicável a grafos direcionados
        }
    }

    // Função para imprimir a árvore de profundidade
    void imprimirArvoreProfundidade() {
        cout << endl; // Linha nova para o output

        vector<vector<pair<int, int>>> adj(numVertices); // Nova lista de adjacência

        // Preenche a lista de adjacência com arestas e IDs
        for (const auto& aresta : arestas) {
            adj[aresta.origem].emplace_back(aresta.destino, aresta.id); // Adiciona aresta
            if (!direcionado) {
                adj[aresta.destino].emplace_back(aresta.origem, aresta.id); // Adiciona aresta reversa
            }
        }

        vector<bool> visited(numVertices, false); // Vetor de visitados
        vector<int> arvoreProfundidade; // Armazena a árvore de profundidade

        dfs(0, visited, adj, arvoreProfundidade); // Executa a DFS para obter a árvore

        // Imprime os IDs das arestas na árvore
        for (int id : arvoreProfundidade) {
            cout << id << " "; // Imprime o ID da aresta
        }
    }

    // Função para impressão da árvore em largura (BFS)
    void arvoreLargura() {
        cout << endl; // Linha nova para o output

        vector<vector<pair<int, int>>> adj(numVertices); // Nova lista de adjacência
        // Preenche a lista de adjacência com arestas e IDs
        for (const auto& aresta : arestas) {
            adj[aresta.origem].emplace_back(aresta.destino, aresta.id);
            if (!direcionado) {
                adj[aresta.destino].emplace_back(aresta.origem, aresta.id);
            }
        }

        vector<bool> visited(numVertices, false); // Vetor de visitados
        queue<int> q; // Fila para a BFS
        vector<int> ordemArestas; // Armazena a ordem das arestas percorridas

        q.push(0); // Inicia na raiz
        visited[0] = true; // Marca como visitado

        while (!q.empty()) { // Enquanto a fila não estiver vazia
            int v = q.front(); // Obtém o vértice do topo
            q.pop(); // Remove o vértice da fila
            sort(adj[v].begin(), adj[v].end()); // Ordena os adjacentes para processamento

            for (const auto& p : adj[v]) { // Para cada vizinho
                int u = p.first; // Obtém o vértice adjacente
                int id = p.second; // Obtém o ID da aresta
                if (!visited[u]) { // Se não visitado
                    visited[u] = true; // Marca como visitado
                    ordemArestas.push_back(id); // Adiciona o ID da aresta à ordem
                    q.push(u); // Adiciona o adjacente à fila
                }
            }
        }

        for (int id : ordemArestas) { // Imprime os IDs das arestas na BFS
            cout << id << " "; // Imprime o ID da aresta
        }
    }

    // Função para encontrar a árvore geradora mínima
    void arvoreGeradoraMinima() {
        cout << endl; // Linha nova para o output

        if (direcionado) {
            cout << "-1"; // Não aplicável a grafos direcionados
            return; 
        }

        // Ordena as arestas pelo peso
        sort(arestas.begin(), arestas.end(), [](const Aresta& a, const Aresta& b) {
            return a.peso < b.peso; // Compara os pesos das arestas
        });

        vector<int> pai(numVertices); // Vetor para o pai de cada vértice
        for (int i = 0; i < numVertices; ++i) {
            pai[i] = i; // Inicializa o pai como o próprio vértice
        }

        // Função auxiliar para encontrar o representante de um conjunto
        function<int(int)> encontrar = [&](int u) {
            return pai[u] == u ? u : pai[u] = encontrar(pai[u]); // Busca com compressão de caminho
        };

        int valorMST = 0; // Valor total da árvore geradora mínima

        // Para cada aresta, verifica se pode ser adicionada à MST
        for (const auto& aresta : arestas) {
            int u = encontrar(aresta.origem); // Encontra o conjunto do vértice de origem
            int v = encontrar(aresta.destino); // Encontra o conjunto do vértice de destino
            if (u != v) { // Se não estão no mesmo conjunto
                valorMST += aresta.peso; // Adiciona o peso à soma total
                pai[u] = v; // Une os conjuntos
            }
        }

        cout << valorMST; // Imprime o valor total da árvore geradora mínima
    }

    // Função para imprimir a ordenação topológica
    void imprimirOrdenacaoTopologica() {
        cout << endl; // Linha nova para o output
        if (!direcionado) {
            cout << "-1"; // Não aplicável a grafos não direcionados
            return; 
        }

        // Preenche a lista de adjacência com arestas direcionadas
        for (const auto& aresta : arestas) {
            adj[aresta.origem].push_back(aresta.destino);
        }

        // Ordena os adjacentes para processamento
        for (int i = 0; i < numVertices; ++i) {
            sort(adj[i].begin(), adj[i].end());
        }

        vector<bool> visited(numVertices, false); // Vetor de visitados
        vector<int> ordenacaoTopologica; // Armazena a ordenação topológica

        // Função para realizar DFS
        function<void(int)> dfs = [&](int v) {
            visited[v] = true; // Marca o vértice como visitado
            for (int u : adj[v]) { // Para cada adjacente
                if (!visited[u]) { // Se não visitado
                    dfs(u); // Chama DFS recursivamente
                }
            }
            ordenacaoTopologica.push_back(v); // Adiciona à ordenação
        };

        // Para cada vértice, chama DFS
        for (int i = 0; i < numVertices; ++i) {
            if (!visited[i]) {
                dfs(i); // Chama DFS se não visitado
            }
        }

        reverse(ordenacaoTopologica.begin(), ordenacaoTopologica.end()); // Inverte para obter a ordenação correta

        // Imprime a ordenação topológica
        for (int v : ordenacaoTopologica) {
            cout << v << " "; // Imprime o vértice na ordenação
        }
    }

    // Função para encontrar o caminho mínimo usando Dijkstra
    void caminhoMinimo() {
        cout << endl; // Linha nova para o output
        vector<vector<pair<int, int>>> adj(numVertices); // Nova lista de adjacência

        // Preenche a lista de adjacência com arestas e pesos
        for (const auto& aresta : arestas) {
            adj[aresta.origem].emplace_back(aresta.destino, aresta.peso);
            if (!direcionado) {
                adj[aresta.destino].emplace_back(aresta.origem, aresta.peso);
            }
        }

        vector<int> dist(numVertices, numeric_limits<int>::max()); // Inicializa distâncias como infinito
        dist[0] = 0; // Distância para a origem é 0

        priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq; // Fila de prioridade
        pq.push({0, 0}); // Insere o par (distância, vértice)

        // Enquanto a fila não estiver vazia
        while (!pq.empty()) {
            int d = pq.top().first; // Obtém a distância mais curta
            int v = pq.top().second; // Obtém o vértice correspondente
            pq.pop(); // Remove o elemento da fila

            if (d > dist[v]) continue; // Se já encontrou uma distância menor, continua

            for (const auto& p : adj[v]) { // Para cada adjacente
                int u = p.first; // Vértice adjacente
                int peso = p.second; // Peso da aresta
                if (dist[v] + peso < dist[u]) { // Relaxamento da aresta
                    dist[u] = dist[v] + peso; // Atualiza a distância
                    pq.push({dist[u], u}); // Adiciona à fila
                }
            }
        }
        cout << dist[numVertices - 1]; // Imprime a distância do vértice 0 ao último vértice
    }

    // Função para calcular o fluxo máximo usando o algoritmo de Ford-Fulkerson
    void fluxoMaximo() {
        cout << endl; // Linha nova para o output

        if (!direcionado) {
            cout << "-1"; // Não aplicável a grafos não direcionados
            return; 
        }

        vector<vector<int>> capacidade(numVertices, vector<int>(numVertices, 0)); // Matrizes de capacidade

        // Preenche a matriz de capacidades e lista de adjacência
        for (const auto& aresta : arestas) {
            capacidade[aresta.origem][aresta.destino] = aresta.peso; // Define capacidade da aresta
            adj[aresta.origem].push_back(aresta.destino); // Adiciona aresta à lista de adjacência
        }

        // Função auxiliar para executar BFS e encontrar um caminho de aumento
        auto bfs = [&](int s, int t, vector<int>& parent) {
            fill(parent.begin(), parent.end(), -1); // Limpa o vetor de pais
            parent[s] = s; // Marca a fonte como seu próprio pai
            queue<pair<int, int>> q; // Fila para busca
            q.push({s, INT_MAX}); // Inicia a fila com a fonte

            while (!q.empty()) { // Enquanto a fila não estiver vazia
                int cur = q.front().first; // Vértice atual
                int flow = q.front().second; // Fluxo atual
                q.pop(); // Remove da fila

                for (int next : adj[cur]) { // Para cada adjacente
                    if (parent[next] == -1 && capacidade[cur][next] > 0) { // Se pode passar pelo adjacente
                        parent[next] = cur; // Marca o pai
                        int new_flow = min(flow, capacidade[cur][next]); // Calcula novo fluxo
                        if (next == t) { // Se chegou ao destino
                            return new_flow; // Retorna o fluxo encontrado
                        }
                        q.push({next, new_flow}); // Adiciona o próximo vértice à fila
                    }
                }
            }
            return 0; // Retorna 0 se não encontrar mais fluxo
        };

        int s = 0; // Vértice fonte
        int t = numVertices - 1; // Vértice destino
        int fluxo_maximo = 0; // Fluxo máximo encontrado
        vector<int> parent(numVertices); // Vetor de pais

        int new_flow; // Novo fluxo encontrado
        // Enquanto houver fluxo
        while ((new_flow = bfs(s, t, parent)) > 0) {
            fluxo_maximo += new_flow; // Incrementa fluxo máximo
            int cur = t; // A partir do destino
            // Atualiza a matriz de capacidade com o fluxo encontrado
            while (cur != s) {
                int prev = parent[cur]; // Vértice pai
                capacidade[prev][cur] -= new_flow; // Diminui capacidade da aresta
                capacidade[cur][prev] += new_flow; // Aumenta capacidade para o fluxo reverso
                cur = prev; // Move para o pai
            }
        }

        cout << fluxo_maximo; // Imprime o fluxo máximo encontrado
    }

    // Função para calcular o fecho transitivo
    void fechoTransitivo() {
        cout << endl; // Linha nova para o output
        if (!direcionado) {
            cout << "-1"; // Não aplicável a grafos não direcionados
            return; 
        }

        // Inicializa matriz de fecho transitivo
        vector<vector<bool>> fecho(numVertices, vector<bool>(numVertices, false));

        // Preenche a matriz de fecho transitivo com arestas
        for (const auto& aresta : arestas) {
            fecho[aresta.origem][aresta.destino] = true; // Marca a aresta
        }

        // Aplica o algoritmo de Floyd-Warshall para calcular o fecho transitivo
        for (int k = 0; k < numVertices; ++k) {
            for (int i = 0; i < numVertices; ++i) {
                for (int j = 0; j < numVertices; ++j) {
                    fecho[i][j] = fecho[i][j] || (fecho[i][k] && fecho[k][j]); // Atualiza matriz
                }
            }
        }

        int verticeEscolhido = 0; // Vértice de referência
        for (int j = 0; j < numVertices; ++j) {
            if (fecho[verticeEscolhido][j]) {
                cout << j << " "; // Imprime todos os vértices acessíveis do escolhido
            }
        }
    }
};

int main() {
    // Lê a entrada do grafo
    int v, a; // Vértices e arestas
    string direcionado; // String para indicar se é direcionado

    cin >> v >> a >> direcionado; // Lê a quantidade de vértices, arestas e se é direcionado
    bool isDirecionado = (direcionado == "direcionado"); // Converte para booleano
    Grafo grafo(v, a, isDirecionado); // Cria o objeto grafo

    for (int i = 0; i < a; ++i) { // Lê cada aresta
        int id, origem, destino, peso; // ID, origem, destino e peso da aresta
        cin >> id >> origem >> destino >> peso; // Lê aresta
        grafo.adicionarAresta(id, origem, destino, peso); // Adiciona aresta ao grafo
    }

    // Executa todas as funcionalidades
    grafo.conexo(); // Verifica conectividade
    cout << endl;

    grafo.bipartido(); // Verifica se é bipartido
    cout << endl;

    grafo.euleriano(); // Verifica se é euleriano
    cout << endl;

    grafo.ciclo(); // Verifica se tem ciclo
    cout << endl;

    grafo.componentesConexas(); // Conta componentes conexas
    cout << endl;

    grafo.componentesFortementeConexas(); // Conta componentes fortemente conexas
    cout << endl;

    grafo.verticesArticulacao(); // Encontra vértices de articulação
    cout << endl;

    grafo.arestasPonte(); // Encontra arestas de ponte
    cout << endl;

    grafo.imprimirArvoreProfundidade(); // Imprime árvore de profundidade
    cout << endl;

    grafo.arvoreLargura(); // Imprime árvore em largura
    cout << endl;

    grafo.arvoreGeradoraMinima(); // Encontra árvore geradora mínima
    cout << endl;

    grafo.imprimirOrdenacaoTopologica(); // Imprime ordenação topológica
    cout << endl;

    grafo.caminhoMinimo(); // Classe caminho mínimo
    cout << endl;

    grafo.fluxoMaximo(); // Encontra fluxo máximo
    cout << endl;

    grafo.fechoTransitivo(); // Calcula fecho transitivo
    cout << endl;

    return 0; // Retorno do programa
}
