// Keyboard movement in the kinematic tree (WAI-ARIA tree view pattern):
// the entries are a flat list in document order, each with its depth.
// DOM-free so it can be tested; the page moves the focus.

/**
 * Index of the entry the key moves to, or -1 when the key does not move.
 * ArrowDown / ArrowUp: next / previous entry; Home / End: first / last;
 * ArrowLeft: the parent (the closest earlier entry with a smaller depth);
 * ArrowRight: the first child (the next entry, when it is deeper).
 * @param {string} key KeyboardEvent.key
 * @param {number} index current entry
 * @param {readonly number[]} levels depth of each entry, in document order
 * @returns {number}
 */
export function treeKeyTarget(key, index, levels) {
  const n = levels.length;
  if (n === 0 || index < 0 || index >= n) return -1;
  switch (key) {
    case "ArrowDown":
      return index + 1 < n ? index + 1 : -1;
    case "ArrowUp":
      return index > 0 ? index - 1 : -1;
    case "Home":
      return index === 0 ? -1 : 0;
    case "End":
      return index === n - 1 ? -1 : n - 1;
    case "ArrowLeft":
      for (let i = index - 1; i >= 0; i--) if (levels[i] < levels[index]) return i;
      return -1;
    case "ArrowRight":
      return index + 1 < n && levels[index + 1] > levels[index] ? index + 1 : -1;
    default:
      return -1;
  }
}
