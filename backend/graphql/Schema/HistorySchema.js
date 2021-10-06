const { gql } = require('apollo-server');

const HistorySchema = gql`
  type Query {
    getHistory: [History]
  }

  type Mutation {
    createHistory(
      event_time: String,
      event_title: String,
      event_desc: String,
      event_img: String
      ): History
    deleteHistory(historyid: Int): History
  }

  type History {
    historyid: Int
    event_time: String
    event_title: String
    event_desc: String
    event_img: String
  }

`;

module.exports = HistorySchema;
