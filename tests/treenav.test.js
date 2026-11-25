import test from "node:test";
import assert from "node:assert/strict";
import { treeKeyTarget } from "../src/core/treenav.js";

// base_link (0) > j1 (1) > arm (2) > j2 (3) > tip (4); base_link > j3 (1) > wheel (2)
const LEVELS = [0, 1, 2, 3, 4, 1, 2];

test("tree keys: up, down, home, end stop at the ends", () => {
  assert.equal(treeKeyTarget("ArrowDown", 0, LEVELS), 1);
  assert.equal(treeKeyTarget("ArrowDown", 6, LEVELS), -1);
  assert.equal(treeKeyTarget("ArrowUp", 3, LEVELS), 2);
  assert.equal(treeKeyTarget("ArrowUp", 0, LEVELS), -1);
  assert.equal(treeKeyTarget("Home", 4, LEVELS), 0);
  assert.equal(treeKeyTarget("Home", 0, LEVELS), -1);
  assert.equal(treeKeyTarget("End", 2, LEVELS), 6);
  assert.equal(treeKeyTarget("End", 6, LEVELS), -1);
});

test("tree keys: left goes to the parent, right to the first child", () => {
  assert.equal(treeKeyTarget("ArrowLeft", 4, LEVELS), 3);
  assert.equal(treeKeyTarget("ArrowLeft", 5, LEVELS), 0);
  assert.equal(treeKeyTarget("ArrowLeft", 0, LEVELS), -1);
  assert.equal(treeKeyTarget("ArrowRight", 0, LEVELS), 1);
  assert.equal(treeKeyTarget("ArrowRight", 4, LEVELS), -1);
  assert.equal(treeKeyTarget("ArrowRight", 6, LEVELS), -1);
});

test("tree keys: other keys and bad indices do nothing", () => {
  assert.equal(treeKeyTarget("a", 1, LEVELS), -1);
  assert.equal(treeKeyTarget("ArrowDown", -1, LEVELS), -1);
  assert.equal(treeKeyTarget("ArrowDown", 7, LEVELS), -1);
  assert.equal(treeKeyTarget("ArrowDown", 0, []), -1);
});
