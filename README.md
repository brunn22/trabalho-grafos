# Algoritmos em Grafos - Trabalho Prático

## Objetivo
Implementar algoritmos para análise de propriedades de grafos, avançando gradualmente à medida que a disciplina progrida. O código deve ser capaz de processar e analisar grafos de diferentes tipos e gerar várias propriedades e estruturas relacionadas.

## Funções a Serem Implementadas

### Verificar
1. **Conexo**: Verifica a conectividade fraca em grafos orientados.
2. **Bipartido**: Verifica se o grafo é bipartido.
3. **Euleriano**: Verifica se o grafo é euleriano.
4. **Possui ciclo**: Verifica se o grafo possui ciclo.

### Listar
5. **Componentes Conexas**: Lista as componentes conexas em ordem lexicográfica.
6. **Componentes Fortemente Conexas**: Lista as componentes fortemente conexas em ordem lexicográfica.
7. **Trilha Euleriana**: Calcula e imprime uma trilha euleriana, priorizando a ordem lexicográfica dos vértices.
8. **Vértices de Articulação**: Identifica e imprime os vértices de articulação.

### Gerar
9. **Identificador das Arestas Ponte**: Gera e imprime os identificadores das arestas ponte.
10. **Árvore de Profundidade**: Gera e imprime a árvore de profundidade, priorizando a ordem lexicográfica dos vértices. Em caso de desconexão, considere apenas a árvore com a raiz 0.
11. **Árvore de Largura**: Gera e imprime a árvore de largura, priorizando a ordem lexicográfica dos vértices. Em caso de desconexão, considere apenas a árvore com a raiz 0.
12. **Árvore Geradora Mínima**: Gera e imprime a árvore geradora mínima, priorizando a ordem lexicográfica dos vértices ou arestas. Aplicável para grafos não-orientados com pelo menos um peso diferente nas arestas.
13. **Ordem Topológica**: Gera e imprime a ordem topológica dos vértices. Esta função não está disponível para grafos não-orientados e deve priorizar a ordem lexicográfica dos vértices.
14. **Valor do Caminho Mínimo entre Dois Vértices**: Calcula e imprime o valor do caminho mínimo entre os vértices 0 e n-1. Aplicável para grafos não-orientados com pelo menos um peso diferente nas arestas.
15. **Valor do Fluxo Máximo**: Calcula e imprime o valor do fluxo máximo entre os vértices 0 e n-1. Esta função não está disponível para grafos não-orientados e deve priorizar a ordem lexicográfica dos vértices.
16. **Fecho Transitivo**: Calcula e imprime o fecho transitivo para o vértice 0. Esta função não está disponível para grafos não-orientados e deve priorizar a ordem lexicográfica dos vértices.

## Exemplo de Caso de Teste (retitado da descrição do trabalho prático)

4 4
nao_direcionado
0 0 1 1
1 1 2 1
2 1 3 1
3 2 3 1


## Nome
Bruno Noberto Gomes
