module.exports = {
  css: {
    loaderOptions: {
      sass: {
        sassOptions: {
          additionalData: `
            @import "@/assets/scss/normalize.scss";
            @import "@/assets/scss/common.scss";
          `
        }
      }
    }
  }
}