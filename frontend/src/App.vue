<template>
  <div id="app">
    <div id="nav">
      <router-link :to="{ name: 'Login' }">로그인</router-link> |
      <router-link :to="{ name: 'Singup' }">회원가입</router-link> |
      <router-link :to="{ name: 'Home' }">홈</router-link>
    </div>
    <router-view/>
  </div>
</template>

<style>
  @import url('./assets/css/normalize.scss');
  #app {
    width: 360px;
    height: 670px;
    margin: 0px auto;
    font-family: Avenir, Helvetica, Arial, sans-serif;
    -webkit-font-smoothing: antialiased;
    -moz-osx-font-smoothing: grayscale;
    text-align: center;
    color: #2c3e50;
    background-color: white;
  }
  body {
    background-color: grey;
  }
  #nav {
    padding: 30px;
  }

  #nav a {
    font-weight: bold;
    color: #2c3e50;
  }

  #nav a.router-link-exact-active {
    color: #42b983;
  }
</style>
