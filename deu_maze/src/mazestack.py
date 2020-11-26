#! /usr/bin/env python
# -*- coding: utf-8 -*-

class Node: # 리스트 형태의 노드 구성
    def __init__(self, data):
        self.data = data
        self.next = None


class Stack: # stack 자료구조
    def __init__(self):
        self.head = None

    def sprint(self):
        for x in self.data:
            print x

    def is_empty(self): # stack이 비어있는지 확인
        if not self.head:
            return True
        return False

    def push(self, data): # stack에 값을 push
        new_node = Node(data)
        new_node.next = self.head
        self.head = new_node

    def pop(self): # stack에 값을 pop
        if self.is_empty(): # stack이 비어있다면 none을 pop
            return None
        ret_data = self.head.data
        self.head = self.head.next
        return ret_data

    def peek(self): # stack의 마지막 값을 미리 볼 수 있다.
        if self.is_empty(): # stack이 비어있다면 none을 return
            return None
        return self.head.data
