import Vue from 'vue'
import App from './App.vue'
import router from './router'
import store from './store'
import VueApollo from 'vue-apollo'
import { ApolloClient } from 'apollo-client'
import { createHttpLink } from 'apollo-link-http'
import { InMemoryCache } from 'apollo-cache-inmemory'
import { provide } from '@vue/composition-api'
import { DefaultApolloClient } from '@vue/apollo-composable'
import { setContext } from '@apollo/client/link/context'
import { FontAwesomeIcon } from '@fortawesome/vue-fontawesome'
import { library } from '@fortawesome/fontawesome-svg-core'
import { onError } from "@apollo/client/link/error";
import { BootstrapVue, IconsPlugin } from 'bootstrap-vue'
import 'bootstrap/dist/css/bootstrap.css'
import 'bootstrap-vue/dist/bootstrap-vue.css'
import { faChevronLeft, faChevronRight, faChevronUp, faChevronDown } from '@fortawesome/free-solid-svg-icons'
import io from 'socket.io-client'

const socket = io(process.env.VUE_APP_BACKEND_SERVER || 'http://localhost:3000'); 
Vue.prototype.$socket = socket;
library.add(faChevronLeft, faChevronRight, faChevronUp, faChevronDown)
Vue.component('font-awesome-icon', FontAwesomeIcon)
Vue.config.productionTip = false
Vue.use(BootstrapVue)
Vue.use(IconsPlugin)


const httpLink = createHttpLink({
  uri: process.env.VUE_APP_GRAPHQL_SERVER || 'http://localhost:4000/graphql',
})

const errorLink = onError(({ graphQLErrors, networkError }) => {
  if (graphQLErrors)
    graphQLErrors.forEach(({ message, locations, path }) =>
      console.log(
        `[GraphQL error]: Message: ${message}, Location: ${locations}, Path: ${path}`,
      ),
    );

  if (networkError) console.log(`[Network error]: ${networkError}`);
});

let token = localStorage.getItem('token') || 'pro'
const setAuthorizationLink = setContext(() => ({
  headers: {
    authorization: token
  }
}));

  
const cache = new InMemoryCache()
const apolloClient = new ApolloClient({
  link: errorLink.concat(setAuthorizationLink.concat(httpLink)),
  cache,
})
Vue.use(VueApollo)
Vue.prototype.$log = console.log

const apolloProvider = new VueApollo({
  defaultClient: apolloClient,
})

new Vue({
  setup () {
    provide(DefaultApolloClient, apolloClient)
  },
  apolloProvider,
  router,
  store,
  render: h => h(App)
}).$mount('#app')
