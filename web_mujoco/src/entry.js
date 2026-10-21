if (new URLSearchParams(window.location.search).get('variant') === 'dual') {
  void import('./dual-main.js');
} else {
  void import('./main.js');
}
