module.exports = function (context, options) {
  return {
    name: 'custom-navbar-plugin',
    getThemePath() {
      return './theme';
    },
  };
};
