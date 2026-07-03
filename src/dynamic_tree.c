// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Sirac Ozmen
//
// Adapted from Box2D's dynamic tree (Copyright 2023 Erin Catto, MIT):
// best-sibling insertion by area heuristic, AVL rotations. All arithmetic
// is f64 +,-,*,compare - inside the allowed op set, deterministic.

#include "dynamic_tree.h"

#include "maul2d/base.h"

static double AabbPerimeter(m2AABB aabb)
{
    double wx = aabb.upperBound.x - aabb.lowerBound.x;
    double wy = aabb.upperBound.y - aabb.lowerBound.y;
    return 2.0 * (wx + wy);
}

static m2AABB AabbUnion(m2AABB a, m2AABB b)
{
    m2AABB c;
    c.lowerBound.x = a.lowerBound.x < b.lowerBound.x ? a.lowerBound.x : b.lowerBound.x;
    c.lowerBound.y = a.lowerBound.y < b.lowerBound.y ? a.lowerBound.y : b.lowerBound.y;
    c.upperBound.x = a.upperBound.x > b.upperBound.x ? a.upperBound.x : b.upperBound.x;
    c.upperBound.y = a.upperBound.y > b.upperBound.y ? a.upperBound.y : b.upperBound.y;
    return c;
}

void m2Tree_Init(m2DynamicTree* tree, m2TreeNode* nodes, int32_t nodeCapacity)
{
    tree->root = M2_NULL_NODE;
    tree->nodeCount = 0;
    tree->nodeCapacity = nodeCapacity;
    tree->freeList = 0;
    for (int32_t i = 0; i < nodeCapacity; ++i)
    {
        nodes[i] = (m2TreeNode){0};
        nodes[i].parentOrNext = i + 1 < nodeCapacity ? i + 1 : M2_NULL_NODE;
        nodes[i].height = -1;
    }
}

static int32_t AllocateNode(m2DynamicTree* tree, m2TreeNode* nodes)
{
    if (tree->freeList == M2_NULL_NODE)
    {
        return M2_NULL_NODE;
    }
    int32_t node = tree->freeList;
    tree->freeList = nodes[node].parentOrNext;
    nodes[node] = (m2TreeNode){0};
    nodes[node].parentOrNext = M2_NULL_NODE;
    nodes[node].child1 = M2_NULL_NODE;
    nodes[node].child2 = M2_NULL_NODE;
    nodes[node].flags = 1;
    tree->nodeCount += 1;
    return node;
}

static void FreeNode(m2DynamicTree* tree, m2TreeNode* nodes, int32_t node)
{
    nodes[node] = (m2TreeNode){0};
    nodes[node].parentOrNext = tree->freeList;
    nodes[node].height = -1;
    tree->freeList = node;
    tree->nodeCount -= 1;
}

// AVL balance: returns the new subtree root (Box2D's rotation scheme).
static int32_t Balance(m2DynamicTree* tree, m2TreeNode* n, int32_t iA)
{
    m2TreeNode* A = n + iA;
    if (A->child1 == M2_NULL_NODE || A->height < 2)
    {
        return iA;
    }

    int32_t iB = A->child1;
    int32_t iC = A->child2;
    m2TreeNode* B = n + iB;
    m2TreeNode* C = n + iC;
    int32_t balance = C->height - B->height;

    if (balance > 1) // rotate C up
    {
        int32_t iF = C->child1;
        int32_t iG = C->child2;
        m2TreeNode* F = n + iF;
        m2TreeNode* G = n + iG;

        C->child1 = iA;
        C->parentOrNext = A->parentOrNext;
        A->parentOrNext = iC;
        if (C->parentOrNext != M2_NULL_NODE)
        {
            if (n[C->parentOrNext].child1 == iA)
            {
                n[C->parentOrNext].child1 = iC;
            }
            else
            {
                n[C->parentOrNext].child2 = iC;
            }
        }
        else
        {
            tree->root = iC;
        }

        if (F->height > G->height)
        {
            C->child2 = iF;
            A->child2 = iG;
            G->parentOrNext = iA;
            A->aabb = AabbUnion(B->aabb, G->aabb);
            C->aabb = AabbUnion(A->aabb, F->aabb);
            A->height = 1 + (B->height > G->height ? B->height : G->height);
            C->height = 1 + (A->height > F->height ? A->height : F->height);
        }
        else
        {
            C->child2 = iG;
            A->child2 = iF;
            F->parentOrNext = iA;
            A->aabb = AabbUnion(B->aabb, F->aabb);
            C->aabb = AabbUnion(A->aabb, G->aabb);
            A->height = 1 + (B->height > F->height ? B->height : F->height);
            C->height = 1 + (A->height > G->height ? A->height : G->height);
        }
        return iC;
    }

    if (balance < -1) // rotate B up
    {
        int32_t iD = B->child1;
        int32_t iE = B->child2;
        m2TreeNode* D = n + iD;
        m2TreeNode* E = n + iE;

        B->child1 = iA;
        B->parentOrNext = A->parentOrNext;
        A->parentOrNext = iB;
        if (B->parentOrNext != M2_NULL_NODE)
        {
            if (n[B->parentOrNext].child1 == iA)
            {
                n[B->parentOrNext].child1 = iB;
            }
            else
            {
                n[B->parentOrNext].child2 = iB;
            }
        }
        else
        {
            tree->root = iB;
        }

        if (D->height > E->height)
        {
            B->child2 = iD;
            A->child1 = iE;
            E->parentOrNext = iA;
            A->aabb = AabbUnion(C->aabb, E->aabb);
            B->aabb = AabbUnion(A->aabb, D->aabb);
            A->height = 1 + (C->height > E->height ? C->height : E->height);
            B->height = 1 + (A->height > D->height ? A->height : D->height);
        }
        else
        {
            B->child2 = iE;
            A->child1 = iD;
            D->parentOrNext = iA;
            A->aabb = AabbUnion(C->aabb, D->aabb);
            B->aabb = AabbUnion(A->aabb, E->aabb);
            A->height = 1 + (C->height > D->height ? C->height : D->height);
            B->height = 1 + (A->height > E->height ? A->height : E->height);
        }
        return iB;
    }

    return iA;
}

static void FixUpward(m2DynamicTree* tree, m2TreeNode* nodes, int32_t index)
{
    while (index != M2_NULL_NODE)
    {
        index = Balance(tree, nodes, index);
        m2TreeNode* node = nodes + index;
        int32_t c1 = node->child1;
        int32_t c2 = node->child2;
        node->aabb = AabbUnion(nodes[c1].aabb, nodes[c2].aabb);
        node->height =
            1 + (nodes[c1].height > nodes[c2].height ? nodes[c1].height : nodes[c2].height);
        index = node->parentOrNext;
    }
}

int32_t m2Tree_Insert(m2DynamicTree* tree, m2TreeNode* nodes, m2AABB aabb, int32_t userData)
{
    int32_t leaf = AllocateNode(tree, nodes);
    if (leaf == M2_NULL_NODE)
    {
        return M2_NULL_NODE;
    }
    nodes[leaf].aabb = aabb;
    nodes[leaf].userData = userData;
    nodes[leaf].height = 0;

    if (tree->root == M2_NULL_NODE)
    {
        tree->root = leaf;
        nodes[leaf].parentOrNext = M2_NULL_NODE;
        return leaf;
    }

    // Find the best sibling by the surface-area heuristic.
    int32_t index = tree->root;
    while (nodes[index].height > 0)
    {
        int32_t child1 = nodes[index].child1;
        int32_t child2 = nodes[index].child2;

        double area = AabbPerimeter(nodes[index].aabb);
        double combinedArea = AabbPerimeter(AabbUnion(nodes[index].aabb, aabb));
        double cost = 2.0 * combinedArea;
        double inheritanceCost = 2.0 * (combinedArea - area);

        double cost1;
        m2AABB aabb1 = AabbUnion(aabb, nodes[child1].aabb);
        if (nodes[child1].height == 0)
        {
            cost1 = AabbPerimeter(aabb1) + inheritanceCost;
        }
        else
        {
            cost1 = AabbPerimeter(aabb1) - AabbPerimeter(nodes[child1].aabb) + inheritanceCost;
        }

        double cost2;
        m2AABB aabb2 = AabbUnion(aabb, nodes[child2].aabb);
        if (nodes[child2].height == 0)
        {
            cost2 = AabbPerimeter(aabb2) + inheritanceCost;
        }
        else
        {
            cost2 = AabbPerimeter(aabb2) - AabbPerimeter(nodes[child2].aabb) + inheritanceCost;
        }

        if (cost < cost1 && cost < cost2)
        {
            break;
        }
        index = cost1 < cost2 ? child1 : child2;
    }

    int32_t sibling = index;
    int32_t oldParent = nodes[sibling].parentOrNext;
    int32_t newParent = AllocateNode(tree, nodes);
    if (newParent == M2_NULL_NODE)
    {
        // Pool exhausted mid-insert: undo the leaf, fail loudly upstream.
        FreeNode(tree, nodes, leaf);
        return M2_NULL_NODE;
    }
    nodes[newParent].parentOrNext = oldParent;
    nodes[newParent].aabb = AabbUnion(aabb, nodes[sibling].aabb);
    nodes[newParent].height = nodes[sibling].height + 1;
    nodes[newParent].child1 = sibling;
    nodes[newParent].child2 = leaf;
    nodes[sibling].parentOrNext = newParent;
    nodes[leaf].parentOrNext = newParent;

    if (oldParent != M2_NULL_NODE)
    {
        if (nodes[oldParent].child1 == sibling)
        {
            nodes[oldParent].child1 = newParent;
        }
        else
        {
            nodes[oldParent].child2 = newParent;
        }
    }
    else
    {
        tree->root = newParent;
    }

    FixUpward(tree, nodes, nodes[leaf].parentOrNext);
    return leaf;
}

void m2Tree_Remove(m2DynamicTree* tree, m2TreeNode* nodes, int32_t proxy)
{
    M2_ASSERT(proxy >= 0 && proxy < tree->nodeCapacity && nodes[proxy].height == 0);

    if (tree->root == proxy)
    {
        tree->root = M2_NULL_NODE;
        FreeNode(tree, nodes, proxy);
        return;
    }

    int32_t parent = nodes[proxy].parentOrNext;
    int32_t grandParent = nodes[parent].parentOrNext;
    int32_t sibling = nodes[parent].child1 == proxy ? nodes[parent].child2 : nodes[parent].child1;

    if (grandParent != M2_NULL_NODE)
    {
        if (nodes[grandParent].child1 == parent)
        {
            nodes[grandParent].child1 = sibling;
        }
        else
        {
            nodes[grandParent].child2 = sibling;
        }
        nodes[sibling].parentOrNext = grandParent;
        FreeNode(tree, nodes, parent);
        FixUpward(tree, nodes, grandParent);
    }
    else
    {
        tree->root = sibling;
        nodes[sibling].parentOrNext = M2_NULL_NODE;
        FreeNode(tree, nodes, parent);
    }
    FreeNode(tree, nodes, proxy);
}

void m2Tree_Move(m2DynamicTree* tree, m2TreeNode* nodes, int32_t proxy, m2AABB aabb)
{
    int32_t userData = nodes[proxy].userData;
    m2Tree_Remove(tree, nodes, proxy);
    int32_t fresh = m2Tree_Insert(tree, nodes, aabb, userData);
    // Same node index comes back: Remove pushed exactly the nodes Insert
    // pops (LIFO free list), and the proxy was freed last.
    M2_ASSERT(fresh == proxy);
    (void)fresh;
}

int32_t m2Tree_Query(const m2DynamicTree* tree, const m2TreeNode* nodes, m2AABB aabb,
                     int32_t* results, int32_t resultCapacity)
{
    int32_t stack[256];
    int32_t top = 0;
    int32_t count = 0;
    if (tree->root != M2_NULL_NODE)
    {
        stack[top++] = tree->root;
    }
    while (top > 0)
    {
        int32_t index = stack[--top];
        if (!m2AABB_Overlaps(nodes[index].aabb, aabb))
        {
            continue;
        }
        if (nodes[index].height == 0)
        {
            if (count < resultCapacity)
            {
                results[count] = nodes[index].userData;
            }
            count += 1;
        }
        else
        {
            M2_ASSERT(top + 2 <= 256);
            stack[top++] = nodes[index].child1;
            stack[top++] = nodes[index].child2;
        }
    }
    return count;
}

static bool ValidateNode(const m2DynamicTree* tree, const m2TreeNode* nodes, int32_t index)
{
    if (index == M2_NULL_NODE)
    {
        return true;
    }
    const m2TreeNode* node = nodes + index;
    if (node->height == 0)
    {
        return node->child1 == M2_NULL_NODE && node->child2 == M2_NULL_NODE;
    }
    int32_t c1 = node->child1;
    int32_t c2 = node->child2;
    if (c1 < 0 || c1 >= tree->nodeCapacity || c2 < 0 || c2 >= tree->nodeCapacity)
    {
        return false;
    }
    if (nodes[c1].parentOrNext != index || nodes[c2].parentOrNext != index)
    {
        return false;
    }
    int32_t expected =
        1 + (nodes[c1].height > nodes[c2].height ? nodes[c1].height : nodes[c2].height);
    if (node->height != expected)
    {
        return false;
    }
    if (!m2AABB_Contains(node->aabb, nodes[c1].aabb) ||
        !m2AABB_Contains(node->aabb, nodes[c2].aabb))
    {
        return false;
    }
    return ValidateNode(tree, nodes, c1) && ValidateNode(tree, nodes, c2);
}

bool m2Tree_Validate(const m2DynamicTree* tree, const m2TreeNode* nodes)
{
    if (tree->root != M2_NULL_NODE && nodes[tree->root].parentOrNext != M2_NULL_NODE)
    {
        return false;
    }
    return ValidateNode(tree, nodes, tree->root);
}
